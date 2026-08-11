#!/usr/bin/env python3
"""Static scan of the FPMS-OS shell for `set -e` landmines. No Pi, no image.

    python3 selftest/test_shell_hazards.py

THE CHECK THAT JUSTIFIES THIS FILE is `sigpipe_in_status_bearing_context`.
This one bug class has now killed two builds, and cost fourteen hours the
first time.

    cand="$(apt-cache policy "$p" | awk '/Candidate:/{print $2; exit}')"

Under `set -euo pipefail`, awk's `exit` closes the pipe while apt-cache is
still writing. apt-cache dies of SIGPIPE, which is exit 141. `pipefail` makes
the whole pipeline 141. The command substitution carries 141 out. And a PLAIN
assignment ADOPTS the status of its last substitution -- so `set -e` kills the
script on that line.

Three things make it nearly impossible to catch by reading:

  * `local cand="$(...)"` MASKS IT ENTIRELY, because `local` is itself a
    command and supplies its own exit status. The identical pipeline is fatal
    on one line and harmless on the next, and nothing about the two lines
    looks different.
  * It prints NOTHING. `set -e` kills the shell before any handler runs. The
    real symptom was "FAILED after 0 min" and a log ending mid-sentence.
  * It is TIMING-DEPENDENT. If the producer's whole output fits in the 64 KiB
    pipe buffer before the consumer exits, there is no SIGPIPE and the line
    works. So it passes in testing and fails in production, and it can come
    back to life when a package list grows.

The two live instances (stage 10's `| grep -q` repo gate and stage 10's
`apt_group` `| awk '...exit'`) are both fixed. This file exists so they cannot
come back, and so the same shape cannot arrive in a new stage unnoticed.

The analyser is deliberately biased toward silence: a checker that cries wolf
gets ignored, so every rule below is written to under-report rather than
over-report. See LIMITS at the bottom of the output for exactly what it
cannot see.

Every run first re-proves the analyser against a built-in fixture set -- the
two real bugs, the `local` variant that must NOT fire, the `|| true` variant
that must NOT fire, and a clean file. If the analyser cannot pass its own
fixtures, this file fails before it reports anything about the real tree.
"""

import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

failures, warnings, infos = [], [], []


def fail(check, msg):
    failures.append((check, msg))


def warn(check, msg):
    warnings.append((check, msg))


def info(check, msg):
    infos.append((check, msg))


# ===========================================================================
# A small, quote-aware shell scanner.
#
# Everything here exists to avoid false positives. Naive line matching flags
# `cmd "journalctl -u fpms-cored | grep -i stop | tail"` in fpms-doctor, where
# the pipeline is a STRING passed to a helper and never runs as a pipeline at
# this point at all. It also mangles `# awk's exit` -- an apostrophe in a
# comment -- into an open single quote that swallows the rest of the file.
# So: track quotes, track $( ) nesting, strip comments, skip heredoc bodies,
# and join both backslash continuations and multi-line quoted strings.
# ===========================================================================

class Scanner:
    """Incremental shell lexer state, carried across physical lines."""

    def __init__(self):
        self.stack = []   # '$(' | '(' | '`' | '$((' | '${'
        self.qs = []      # "'" | '"'

    def clean(self):
        return not self.stack and not self.qs

    def in_squote(self):
        return bool(self.qs) and self.qs[-1] == "'"

    def feed(self, line):
        """Consume one physical line up to any comment.

        Returns (code, depth, quote, esc): the code portion of the line and
        one entry per character of it. depth is $( ) / ( ) nesting, quote is
        the innermost quote character or None, esc marks backslash escapes.
        The comment tail is never fed, so an apostrophe in prose cannot open
        a string.
        """
        n = len(line)
        depth, quote, esc = [], [], []
        i = 0
        while i < n:
            c = line[i]
            cq = self.qs[-1] if self.qs else None
            d = len(self.stack)

            # A '#' starts a comment only unquoted, at top nesting level, and
            # at a word boundary -- so `${v#pfx}`, `x=a#b` and `$#` survive.
            if (c == "#" and cq is None and d == 0
                    and (i == 0 or line[i - 1] in " \t;|&(")):
                break

            depth.append(d)
            quote.append(cq)
            esc.append(False)

            if cq == "'":
                if c == "'":
                    self.qs.pop()
                i += 1
                continue

            if c == "\\":
                esc[-1] = True
                if i + 1 < n:
                    depth.append(d)
                    quote.append(cq)
                    esc.append(True)
                    i += 2
                else:
                    i += 1
                continue

            if c == '"':
                if cq == '"':
                    self.qs.pop()
                else:
                    self.qs.append('"')
                i += 1
                continue

            if cq == '"':
                # Inside double quotes only substitutions reopen nesting.
                if line.startswith("$(", i):
                    self.stack.append("$(")
                    depth.append(d)
                    quote.append(cq)
                    esc.append(False)
                    i += 2
                    continue
                if c == "`":
                    self.stack.append("`")
                    i += 1
                    continue
                i += 1
                continue

            if c == "'":
                self.qs.append("'")
                i += 1
                continue
            if line.startswith("$((", i):
                self.stack.append("$((")
                for _ in range(2):
                    depth.append(d)
                    quote.append(cq)
                    esc.append(False)
                i += 3
                continue
            if line.startswith("${", i):
                self.stack.append("${")
                depth.append(d)
                quote.append(cq)
                esc.append(False)
                i += 2
                continue
            if line.startswith("$(", i):
                self.stack.append("$(")
                depth.append(d)
                quote.append(cq)
                esc.append(False)
                i += 2
                continue
            if c == "`":
                if self.stack and self.stack[-1] == "`":
                    self.stack.pop()
                else:
                    self.stack.append("`")
                i += 1
                continue
            if c == "(":
                self.stack.append("(")
                i += 1
                continue
            if c == ")":
                if self.stack and self.stack[-1] == "$((" and line.startswith("))", i):
                    self.stack.pop()
                    depth.append(d)
                    quote.append(cq)
                    esc.append(False)
                    i += 2
                    continue
                if self.stack and self.stack[-1] in ("$(", "("):
                    self.stack.pop()
                i += 1
                continue
            if c == "}":
                if self.stack and self.stack[-1] == "${":
                    self.stack.pop()
                i += 1
                continue
            i += 1

        code = line[:len(depth)] if len(depth) < n else line
        # len(depth) can exceed len(code) by construction (multi-char tokens
        # push several entries); trim to match.
        code = line[:i] if i <= n else line
        depth = depth[:len(code)]
        quote = quote[:len(code)]
        esc = esc[:len(code)]
        while len(depth) < len(code):
            depth.append(0)
            quote.append(None)
            esc.append(False)
        return code, depth, quote, esc


HEREDOC_RX = re.compile(r"<<-?\s*(\\?)(['\"]?)([A-Za-z_][A-Za-z0-9_]*)\2")


class Logical(object):
    """One logical line: text, plus a physical line number per character."""

    def __init__(self, text, linemap, start):
        self.text = text
        self.linemap = linemap
        self.start = start

    def lineno(self, offset):
        if not self.linemap:
            return self.start
        return self.linemap[min(offset, len(self.linemap) - 1)]


def logical_lines(source):
    """Split a shell file into logical lines.

    Comments removed, heredoc bodies removed, backslash continuations joined,
    and multi-line quoted strings joined -- stage 40 ends an awk program four
    lines below where it starts, and the tail of it reads `' "$x" | grep -v`,
    which means nothing on its own.
    """
    sc = Scanner()
    out = []
    buf, lmap, start = [], [], None
    pending = []          # heredoc delimiters still open
    lines = source.splitlines()
    i = 0
    while i < len(lines):
        raw = lines[i]
        i += 1
        if pending:
            stripped = raw.strip() if pending[0][1] else raw.rstrip()
            if stripped == pending[0][0]:
                pending.pop(0)
            continue

        code, depth, quote, esc = sc.feed(raw)

        cont = False
        if code.endswith("\\") and not sc.in_squote():
            # unescaped trailing backslash -> line continuation
            k = len(code) - 1
            back = 0
            while k >= 0 and code[k] == "\\":
                back += 1
                k -= 1
            if back % 2 == 1:
                cont = True
                code = code[:-1]

        if start is None and code.strip():
            start = i
        if start is None:
            start = i
        buf.append(code)
        lmap.extend([i] * len(code))
        buf.append(" " if cont else "\n")
        lmap.append(i)

        # Heredocs open at the end of the physical line that names them.
        for m in HEREDOC_RX.finditer(code):
            p = m.start()
            if p < len(quote) and (quote[p] is not None or depth[p] != 0):
                continue
            if code[p:p + 3] == "<<<":
                continue
            pending.append((m.group(3), code[m.start():m.start() + 3].startswith("<<-")))

        if not cont and sc.clean() and not pending:
            text = "".join(buf)
            if text.strip():
                out.append(Logical(text, lmap, start))
            buf, lmap, start = [], [], None

    if buf and "".join(buf).strip():
        out.append(Logical("".join(buf), lmap, start or len(lines)))
    return out


# --------------------------------------------------------------------------
# Splitting a logical line into and-or lists and pipelines.

def _marks(text):
    """Per-character (depth, quote) for an already-clean logical line."""
    sc = Scanner()
    _, depth, quote, esc = sc.feed(text.replace("\n", " "))
    while len(depth) < len(text):
        depth.append(0)
        quote.append(None)
        esc.append(False)
    return depth, quote, esc


def _active(text, depth, quote, esc, i):
    return depth[i] == 0 and quote[i] is None and not esc[i]


def split_andor(text):
    """Split into and-or lists on top-level ; & and newline.

    Returns a list of lists of (segment_text, offset, connector_before).
    """
    depth, quote, esc = _marks(text)
    lists, cur = [], []
    seg_start, conn = 0, None
    i = 0
    n = len(text)
    while i < n:
        if not _active(text, depth, quote, esc, i):
            i += 1
            continue
        two = text[i:i + 2]
        if two in ("&&", "||"):
            cur.append((text[seg_start:i], seg_start, conn))
            conn = two
            i += 2
            seg_start = i
            continue
        c = text[i]
        if c in ";\n" or (c == "&" and two != "&&"):
            cur.append((text[seg_start:i], seg_start, conn))
            if any(s.strip() for s, _, _ in cur):
                lists.append(cur)
            cur, conn = [], None
            i += 1
            seg_start = i
            continue
        i += 1
    cur.append((text[seg_start:n], seg_start, conn))
    if any(s.strip() for s, _, _ in cur):
        lists.append(cur)
    return lists


def split_pipeline(text):
    """Split one segment into pipeline elements on top-level single `|`."""
    depth, quote, esc = _marks(text)
    parts, start = [], 0
    i = 0
    n = len(text)
    while i < n:
        if _active(text, depth, quote, esc, i) and text[i] == "|":
            if text[i:i + 2] == "||":
                i += 2
                continue
            if i and text[i - 1] == "|":
                i += 1
                continue
            parts.append((text[start:i], start))
            i += 1
            start = i
            continue
        i += 1
    parts.append((text[start:n], start))
    return parts


def substitutions(text):
    """Top-level $( ) and ` ` bodies: (inner_text, inner_offset)."""
    depth, quote, esc = _marks(text)
    out = []
    i = 0
    n = len(text)
    while i < n:
        if depth[i] == 0 and quote[i] in (None, '"') and not esc[i]:
            if text.startswith("$(", i) and not text.startswith("$((", i):
                j = i + 2
                while j < n and not (depth[j] == 0 and text[j] == ")"):
                    j += 1
                out.append((text[i + 2:j], i + 2))
                i = j + 1
                continue
            if text[i] == "`":
                j = i + 1
                while j < n and not (depth[j] == 0 and text[j] == "`"):
                    j += 1
                out.append((text[i + 1:j], i + 1))
                i = j + 1
                continue
        i += 1
    return out


def words(text):
    """Top-level whitespace-separated words, quotes preserved."""
    depth, quote, esc = _marks(text)
    out, cur, off = [], [], 0
    for i, c in enumerate(text):
        if c in " \t\n" and _active(text, depth, quote, esc, i):
            if cur:
                out.append(("".join(cur), off))
                cur = []
        else:
            if not cur:
                off = i
            cur.append(c)
    if cur:
        out.append(("".join(cur), off))
    return out


def unquote(w):
    return w.replace("'", "").replace('"', "")


def command_name(elem):
    """The command a pipeline element runs, skipping env prefixes/wrappers."""
    for w, _ in words(elem):
        if re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", w):
            continue                       # LC_ALL=C grep ...
        if re.match(r"^\d*[<>]", w):
            continue                       # 2>/dev/null before the command
        base = os.path.basename(unquote(w))
        if base in ("sudo", "command", "env", "time", "exec", "builtin", "nice",
                    "stdbuf", "!"):
            continue
        return base
    return ""


# ===========================================================================
# Rule 1: the SIGPIPE class.
# ===========================================================================

def early_exit_consumer(elem):
    """Why this pipeline element kills its producer, or None if it does not.

    Only consumers that STOP READING BEFORE EOF matter. `grep -v`, `sed -n 1p`
    and an awk with no `exit` all drain their input and are entirely safe --
    stage 25 and stage 0 use exactly those, and flagging them would be noise.
    """
    argv = [unquote(w) for w, _ in words(elem)]
    if not argv:
        return None
    cmd = command_name(elem)
    args = argv[1:] if argv else []

    if cmd in ("grep", "egrep", "fgrep", "rgrep", "zgrep"):
        for a in args:
            if a == "--":
                break
            if a in ("-q", "--quiet", "--silent"):
                return "grep -q exits on the FIRST match"
            if a.startswith("-m") or a.startswith("--max-count"):
                return "grep -m N exits after N matches"
            if a.startswith("-") and not a.startswith("--") and "q" in a[1:]:
                return "grep %s bundles -q, which exits on the first match" % a
        return None

    if cmd in ("head",):
        for a in args:
            # `head -n -5` prints all but the last 5: it reads to EOF.
            if re.match(r"^-n?-\d+$", a) or re.match(r"^--lines=-\d+$", a):
                return None
        return "head stops reading once it has its lines"

    if cmd in ("awk", "gawk", "mawk", "nawk"):
        for a in args:
            if re.search(r"(^|[^A-Za-z_])exit([^A-Za-z0-9_]|$)", a):
                return "awk's `exit` closes the pipe early"
        return None

    if cmd in ("sed",):
        for a in args:
            if a.startswith("-"):
                continue
            if re.search(r"(^|[;{}\s\d/])q([;}\s]|$)", a):
                return "sed's `q` closes the pipe early"
        return None

    if cmd in ("read",):
        return "read consumes one line and returns"

    return None


def _protected(seglist, idx):
    """True if this segment's status is thrown away entirely by `|| true`."""
    if idx + 1 < len(seglist):
        nxt, _, conn = seglist[idx + 1]
        if conn == "||" and nxt.strip().rstrip(";") in ("true", ":", "/bin/true"):
            return True
    return False


def _status_used(seglist, idx):
    """How this segment's exit status is consumed.

    'fatal'    it is the last segment of its list -> `set -e` acts on it
    'branch'   a && or || after it reads it -> the branch goes the wrong way
    'none'     discarded
    """
    if _protected(seglist, idx):
        return "none"
    if idx + 1 < len(seglist):
        return "branch"
    return "fatal"


CONDITION_RX = re.compile(r"^\s*(?:!\s*)?(if|elif|while|until)\s+")
MASKING_RX = re.compile(r"^\s*(local|declare|typeset|export|readonly|eval)\s")
PLAIN_ASSIGN_RX = re.compile(
    r"^\s*(?:[A-Za-z_][A-Za-z0-9_]*(?:\[[^]]*\])?\+?=\S*\s+)*"
    r"([A-Za-z_][A-Za-z0-9_]*(?:\[[^]]*\])?)\+?=")
LEADING_KW_RX = re.compile(
    r"^\s*(?:then|do|else|\{|\(|!|time|elif|if|while|until|for|case|in)\s+")


def _strip_kw(seg):
    """Remove leading shell keywords; return (rest, was_condition)."""
    cond = bool(CONDITION_RX.match(seg))
    prev = None
    while prev != seg:
        prev = seg
        seg = re.sub(r"^\s*(then|do|else|\{|!|time)\s+", "", seg)
        m = CONDITION_RX.match(seg)
        if m:
            seg = seg[m.end():]
            cond = True
    return seg, cond


def scan_sigpipe(path, lls, opts):
    """Rule 1. Report only where the status is actually load-bearing."""
    rel = os.path.relpath(path, ROOT).replace("\\", "/")
    if not opts["pipefail_ever"]:
        return

    def check_segment(ll, seg, off, kind, use, eflag):
        elems = split_pipeline(seg)
        if len(elems) < 2:
            return
        last, loff = elems[-1]
        why = early_exit_consumer(last)
        if not why:
            return
        ln = ll.lineno(off + loff)
        snippet = " ".join(seg.split())
        if len(snippet) > 110:
            snippet = snippet[:107] + "..."
        where = {
            "assign": ("a PLAIN assignment, which ADOPTS the pipeline's exit "
                       "status (a `local`/`export` here would mask it)"),
            "cond": ("an if/while condition -- `set -e` spares it, but "
                     "pipefail still turns a SUCCESSFUL match into 141, so "
                     "the test fires exactly BACKWARDS"),
            "bare": "a bare statement, whose status `set -e` acts on",
        }[kind]
        msg = ("%s:%d: %s\n"
               "          %s\n"
               "          %s in %s."
               % (rel, ln, snippet, why, "producer dies of SIGPIPE (141)", where))
        if use == "none":
            return
        if kind == "cond":
            fail("sigpipe", msg)
        elif not eflag:
            warn("sigpipe", msg + "\n          (`set -e` looks disabled here, "
                                  "so this corrupts a status rather than "
                                  "killing the script.)")
        elif use == "branch":
            fail("sigpipe", msg + "\n          The following %s reads that "
                                  "status, so the branch goes the wrong way."
                 % "&&/||")
        else:
            fail("sigpipe", msg)

    for ll in lls:
        eflag = opts["e_at"](ll.start)
        pf = opts["pf_at"](ll.start)
        if not pf:
            continue
        for seglist in split_andor(ll.text):
            for idx, (seg, off, _conn) in enumerate(seglist):
                if not seg.strip():
                    continue
                use = _status_used(seglist, idx)
                if use == "none":
                    continue
                rest, is_cond = _strip_kw(seg)
                roff = off + (len(seg) - len(rest))

                if is_cond:
                    check_segment(ll, rest, roff, "cond", use, eflag)
                    # A substitution inside a condition (`if [ "$(a|head -1)" ]`)
                    # has its status discarded by `[`, so it is not checked.
                    continue

                if MASKING_RX.match(rest):
                    continue      # local/declare/export supply their own status

                m = PLAIN_ASSIGN_RX.match(rest)
                if m:
                    for inner, ioff in substitutions(rest):
                        for sublist in split_andor(inner):
                            for j, (s2, o2, _c) in enumerate(sublist):
                                if _status_used(sublist, j) == "none":
                                    continue
                                if j + 1 < len(sublist):
                                    continue   # only the last decides $?
                                check_segment(ll, s2, roff + ioff + o2,
                                              "assign", use, eflag)
                    continue

                check_segment(ll, rest, roff, "bare", use, eflag)


# ===========================================================================
# Rule 2: the other `set -e` landmines.
# ===========================================================================

ARITH_RX = re.compile(r"\(\(\s*([^)]*)\s*\)\)")
CLOSERS = ("}", "done", "fi", "esac", "))")


def scan_landmines(path, lls, opts):
    rel = os.path.relpath(path, ROOT).replace("\\", "/")
    prev_stmt = None            # (text, lineno) of the previous statement
    block = []                  # stack of 'func' | 'loop' | 'other'

    for ll in lls:
        eflag = opts["e_at"](ll.start)
        for seglist in split_andor(ll.text):
            for idx, (seg, off, _conn) in enumerate(seglist):
                body = seg.strip()
                if not body:
                    continue
                ln = ll.lineno(off)
                use = _status_used(seglist, idx)
                rest, is_cond = _strip_kw(seg)
                rest = rest.strip()
                snippet = " ".join(body.split())[:110]

                # -- block bookkeeping ---------------------------------------
                if re.match(r"^(\w[\w:.-]*\s*\(\)\s*\{?|function\s+\w+)", body):
                    block.append("func")
                elif re.match(r"^(for|while|until)\b", body) or body == "do":
                    if not (block and block[-1] == "loop_pending"):
                        block.append("loop")
                elif body.startswith(("}", "done")):
                    kind = block.pop() if block else None
                    if prev_stmt and kind in ("func", "loop"):
                        ptext, pln, puse = prev_stmt
                        if re.match(r"^\s*(\[\[?|test)\b", ptext) and puse == "branch":
                            fail("landmine",
                                 "%s:%d: %s\n"
                                 "          `[ ... ] && ...` is the last "
                                 "statement of this %s, so the %s adopts the "
                                 "test's status. When the test is false the "
                                 "%s returns 1 -- and under `set -e` that is "
                                 "fatal at the CALL SITE, far from here."
                                 % (rel, pln, ptext[:110],
                                    "function" if kind == "func" else "loop body",
                                    "function" if kind == "func" else "loop",
                                    "function" if kind == "func" else "loop"))
                        if re.match(r"^\s*while\s+(read|IFS=)", ptext) and kind == "func":
                            warn("landmine",
                                 "%s:%d: %s\n"
                                 "          a `while read` loop as the last "
                                 "statement of a function returns the status "
                                 "of the loop BODY's last command, not the "
                                 "loop's. Add an explicit `return 0`."
                                 % (rel, pln, ptext[:110]))

                # -- (( i++ )) -----------------------------------------------
                if use != "none" and eflag:
                    bare_arith = re.match(r"^\(\((.*)\)\)\s*$", rest)
                    if bare_arith:
                        expr = bare_arith.group(1)
                        risky = (re.search(r"(\+\+|--)", expr)
                                 or re.match(r"^\s*\w+\s*(=|\+=|-=|\*=)\s*", expr))
                        if risky:
                            fail("landmine",
                                 "%s:%d: %s\n"
                                 "          `(( ))` returns 1 when the "
                                 "expression EVALUATES TO 0. `((i++))` with "
                                 "i==0 returns 1 and `set -e` kills the "
                                 "script. Use `i=$((i+1))` or `((i++)) || :`."
                                 % (rel, ln, snippet))

                    if re.match(r"^let(\s|$)", rest):
                        fail("landmine",
                             "%s:%d: %s\n"
                             "          bare `let` returns 1 when its last "
                             "expression is 0 -- the same trap as `((i++))`, "
                             "with no arithmetic-context excuse. Use "
                             "`x=$((...))`."
                             % (rel, ln, snippet))

                    if command_name(rest) == "expr":
                        fail("landmine",
                             "%s:%d: %s\n"
                             "          `expr` exits 1 when its result is 0 "
                             "or the empty string, so a perfectly correct "
                             "computation of zero is fatal under `set -e`. "
                             "Use `$(( ))`."
                             % (rel, ln, snippet))

                    # bare grep with nothing consuming its status
                    if (not is_cond and use == "fatal"
                            and not PLAIN_ASSIGN_RX.match(rest)
                            and not MASKING_RX.match(rest)):
                        elems = split_pipeline(rest)
                        if command_name(elems[-1][0]) in ("grep", "egrep", "fgrep"):
                            info("landmine",
                                 "%s:%d: %s\n"
                                 "          a bare `grep` exits 1 when it "
                                 "matches NOTHING, and `set -e` treats that "
                                 "as failure. If no match is a normal "
                                 "outcome here, add `|| true`."
                                 % (rel, ln, snippet))

                if body not in ("do", "then", "{") and not body.startswith(("}", "done", "fi")):
                    prev_stmt = (body, ln, use)


# ===========================================================================
# Rule 3: -e / pipefail bookkeeping.
# ===========================================================================

# The whole option word list, not just the first flag: `set -euo pipefail`
# puts "pipefail" in a SEPARATE word from "-euo", and a regex that stops at
# "-euo" reports a file as having no pipefail at all -- which would switch off
# the entire SIGPIPE rule everywhere, silently, in exactly the files that
# need it. (That is the first thing this analyser got wrong about itself.)
SET_RX = re.compile(r"(?:^|[;&|(]\s*)set\s+([^;&|\n]*)")


def shell_options(lls):
    """Where -e and pipefail are on, tracked in source order.

    Order tracking matters: build.sh turns `set +e` on around a resize and
    back off again, and stage 10 drops `set +eu` before sourcing ROS's
    setup.bash. Lines in those windows are not under `set -e` at all.
    """
    e_events, pf_events = [(0, False)], [(0, False)]
    e_ever = pf_ever = False
    for ll in lls:
        for m in SET_RX.finditer(ll.text):
            spec = m.group(1)
            ln = ll.lineno(m.start())
            toks = spec.split()
            i = 0
            while i < len(toks):
                t = toks[i]
                if t in ("-o", "+o") and i + 1 < len(toks):
                    if toks[i + 1] == "pipefail":
                        pf_events.append((ln, t == "-o"))
                        pf_ever = pf_ever or t == "-o"
                    i += 2
                    continue
                if t.startswith(("-", "+")) and "o" in t[1:] and i + 1 < len(toks):
                    if toks[i + 1] == "pipefail":
                        pf_events.append((ln, t[0] == "-"))
                        pf_ever = pf_ever or t[0] == "-"
                    if "e" in t[1:]:
                        e_events.append((ln, t[0] == "-"))
                        e_ever = e_ever or t[0] == "-"
                    i += 2
                    continue
                if t.startswith(("-", "+")) and "e" in t[1:]:
                    e_events.append((ln, t[0] == "-"))
                    e_ever = e_ever or t[0] == "-"
                i += 1

    def at(events):
        def f(ln):
            state = False
            for eln, val in events:
                if eln <= ln:
                    state = val
            return state
        return f

    return {
        "e_ever": e_ever,
        "pipefail_ever": pf_ever,
        "e_at": at(e_events),
        "pf_at": (lambda ln: pf_ever) if pf_ever else (lambda ln: False),
        "pf_at_ordered": at(pf_events),
    }


def scan_options(path, opts):
    rel = os.path.relpath(path, ROOT).replace("\\", "/")
    if opts["e_ever"] and not opts["pipefail_ever"]:
        info("shellopts",
             "%s sets -e but NOT pipefail. A failing producer mid-pipeline is "
             "invisible here (`false | tee log` succeeds). The SIGPIPE class "
             "cannot bite -- adding pipefail without auditing the pipelines "
             "first is how it arrives." % rel)
    elif opts["pipefail_ever"] and not opts["e_ever"]:
        info("shellopts",
             "%s sets pipefail but NOT -e. Nothing dies, but a plain "
             "assignment from an early-exiting pipeline still captures the "
             "right text with a 141 status, so any later test of $? or of "
             "this function's return value is wrong." % rel)
    elif not opts["e_ever"] and not opts["pipefail_ever"]:
        info("shellopts",
             "%s sets neither -e nor pipefail; every command failure is "
             "ignored unless explicitly checked." % rel)


# ===========================================================================
# Driving it over a tree.
# ===========================================================================

SHEBANG_RX = re.compile(r"^#!.*\b(bash|sh|dash|ksh|zsh)\b")


def is_shell(path):
    if path.endswith((".py", ".md", ".txt", ".json", ".xml", ".yaml", ".yml")):
        return False
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as fh:
            first = fh.readline()
    except OSError:
        return False
    if SHEBANG_RX.match(first):
        return True
    return path.endswith(".sh")


def targets(root):
    out = []
    b = os.path.join(root, "build.sh")
    if os.path.isfile(b):
        out.append(b)
    for sub in ("scripts", "overlay/usr/local/bin", "overlay/usr/local/sbin"):
        d = os.path.join(root, *sub.split("/"))
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            p = os.path.join(d, f)
            if os.path.isfile(p):
                out.append(p)
    return [p for p in out if is_shell(p)]


def analyse(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        src = fh.read()
    lls = logical_lines(src)
    opts = shell_options(lls)
    scan_options(path, opts)
    scan_sigpipe(path, lls, opts)
    scan_landmines(path, lls, opts)


# ===========================================================================
# The fixtures. The analyser must pass these before it says anything about
# the real tree -- both directions, because a checker that passes broken code
# is worse than no checker.
# ===========================================================================

FIXTURES = {
    # ---- MUST FLAG -------------------------------------------------------
    "bug_grep_q.sh": ("""#!/usr/bin/env bash
set -euo pipefail
# The stage 10 repo gate, as it was when it killed the build.
if apt-cache policy ros-humble-ros-base | grep -q 'Candidate: [0-9]'; then
    echo ok
fi
""", [4]),
    "bug_awk_exit.sh": ("""#!/usr/bin/env bash
set -euo pipefail
apt_group() {
    local p cand
    for p in "$@"; do
        cand="$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/{print $2; exit}')"
        echo "$cand"
    done
}
""", [6]),
    "bug_bare_head.sh": ("""#!/usr/bin/env bash
set -euo pipefail
dpkg -l | head -5
echo "never reached if dpkg outruns head"
""", [3]),
    "bug_continuation.sh": ("""#!/usr/bin/env bash
set -euo pipefail
ver="$(apt-cache policy foo \\
       | awk -F': +' '/Candidate:/{print $2; exit}')"
echo "$ver"
""", [4]),
    "bug_branch.sh": ("""#!/usr/bin/env bash
set -euo pipefail
grep -rn thing . | grep -q needle && echo found
""", [3]),

    # ---- MUST NOT FLAG ---------------------------------------------------
    "ok_local.sh": ("""#!/usr/bin/env bash
set -euo pipefail
f() {
    local cand="$(apt-cache policy foo | awk '/Candidate:/{print $2; exit}')"
    echo "$cand"
}
""", []),
    "ok_or_true.sh": ("""#!/usr/bin/env bash
set -euo pipefail
prev="$(ls -1t ./*.img 2>/dev/null | head -1 || true)"
ver="$(apt-cache policy foo | awk '/Candidate:/{print $2; exit}' || true)"
dpkg -l | grep -q foo || true
echo "${prev:-none} ${ver:-none}"
""", []),
    "ok_no_pipefail.sh": ("""#!/usr/bin/env bash
set -eu
cand="$(apt-cache policy foo | awk '/Candidate:/{print $2; exit}')"
echo "$cand"
""", []),
    "ok_draining.sh": ("""#!/usr/bin/env bash
set -euo pipefail
# None of these consumers stop reading early, so no producer ever gets SIGPIPE.
extra="$(apt-get -s purge -y foo | awk '/^Remv /{print $2}' | grep -vE '^foo$' || true)"
base="$(xz --robot -l x.xz | awk '/^totals/ {print int($5/1048576)}')"
one="$(printf '%s\\n' "$LIST" | sed -n 1p)"
sha="$(sha256sum x | awk '{print $1}')"
echo "$extra $base $one $sha"
""", []),
    "ok_quoted_pipeline.sh": ("""#!/usr/bin/env bash
set -euo pipefail
# fpms-doctor passes pipelines as STRINGS. They are not pipelines here.
cmd() { echo "would run: $1"; }
cmd "journalctl -u fpms-cored -b --no-pager | grep -i stop | tail"
cmd "iw dev | awk '/Interface/{print $2;exit}'"
""", []),
    "ok_comment_apostrophe.sh": ("""#!/usr/bin/env bash
set -euo pipefail
# awk's `exit` is what killed it: don't pipe into grep -q here.
# It used to be: cand="$(apt-cache policy "$p" | awk '/C:/{print $2; exit}')"
pol="$(apt-cache policy foo 2>/dev/null || true)"
case "$pol" in *"Candidate: "*) : ;; esac
""", []),
    "ok_heredoc.sh": ("""#!/usr/bin/env bash
set -euo pipefail
cat > /tmp/doc <<'EOF'
Run this to check:
    apt-cache policy foo | grep -q Candidate
    dpkg -l | head -5
EOF
echo done
""", []),
    "ok_clean.sh": ("""#!/usr/bin/env bash
set -euo pipefail
# The shape the stages use now: capture, then match. No pipeline, no race.
pol="$(apt-cache policy "$1" 2>/dev/null || true)"
case "$pol" in
    *"Candidate: (none)"*|"") echo missing ;;
    *"Candidate: "*)          echo present ;;
esac
""", []),
}


def test_analyser_selfcheck():
    """Prove the analyser fires on the real bugs and stays silent otherwise."""
    global failures, warnings, infos
    tmp = tempfile.mkdtemp(prefix="fpms-shell-hazards-")
    saved = (failures, warnings, infos)
    results = []
    try:
        for name, (body, expect) in sorted(FIXTURES.items()):
            p = os.path.join(tmp, name)
            with open(p, "w", encoding="utf-8") as fh:
                fh.write(body)
            failures, warnings, infos = [], [], []
            with open(p, encoding="utf-8") as fh:
                lls = logical_lines(fh.read())
            opts = shell_options(lls)
            scan_sigpipe(p, lls, opts)
            got = sorted(int(re.search(r":(\d+):", m).group(1))
                         for c, m in failures + warnings if c == "sigpipe")
            results.append((name, sorted(expect), got, got == sorted(expect)))
    finally:
        failures, warnings, infos = saved
        for name in FIXTURES:
            try:
                os.remove(os.path.join(tmp, name))
            except OSError:
                pass
        try:
            os.rmdir(tmp)
        except OSError:
            pass

    print("  fixture proof (analyser checked against known-bad and known-good)")
    for name, expect, got, ok in results:
        print("    %-26s %-9s expect=%-9s got=%s"
              % (name, "PASS" if ok else "*** MISMATCH ***",
                 expect or "[]", got or "[]"))
        if not ok:
            fail("selfcheck",
                 "fixture %s: expected hits on lines %s, got %s. The analyser "
                 "is not trustworthy; fix it before believing anything below."
                 % (name, expect, got))
    print()


# ===========================================================================

LIMITS = """  LIMITS -- what this cannot see, stated so nobody over-trusts it:
    * `set -e` is suspended inside any function called from an if/while
      condition or from the left of &&. A fatal line inside such a function
      is reported here but would not actually kill the build.
    * -e/pipefail state is tracked in SOURCE order. A function defined inside
      a `set +e` window but called outside it is judged by the window.
    * A pipeline built at runtime (eval, a variable holding a command) is
      invisible.
    * A producer whose entire output fits in the 64 KiB pipe buffer never
      raises SIGPIPE, so some reported lines have never yet failed. That is
      the point: they fail later, when the input grows.
    * `awk` is judged by the word `exit` appearing in its program text; an
      `exit` inside a printed string would over-report."""


def main():
    print("=" * 72)
    print(" FPMS-OS shell hazard scan (SIGPIPE + `set -e` landmines)")
    print("=" * 72)

    test_analyser_selfcheck()
    if failures:
        for check, msg in failures:
            print("  [FAIL] %s: %s" % (check, msg))
        print("=" * 72)
        return 1

    files = targets(ROOT)
    for p in files:
        try:
            analyse(p)
        except Exception as exc:                      # noqa: BLE001
            fail("scan", "%s: analyser raised %r"
                 % (os.path.relpath(p, ROOT).replace("\\", "/"), exc))

    print("  %d shell files scanned" % len(files))
    for p in files:
        print("    %s" % os.path.relpath(p, ROOT).replace("\\", "/"))
    print()
    for check, msg in infos:
        print("  [INFO] %s: %s" % (check, msg))
    for check, msg in warnings:
        print("  [WARN] %s: %s" % (check, msg))
    for check, msg in failures:
        print("  [FAIL] %s: %s" % (check, msg))
    print("-" * 72)
    if failures:
        print("  %d FAILED, %d warnings, %d informational"
              % (len(failures), len(warnings), len(infos)))
    else:
        print("  all passed, %d warnings, %d informational"
              % (len(warnings), len(infos)))
    print("-" * 72)
    print(LIMITS)
    print("=" * 72)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
