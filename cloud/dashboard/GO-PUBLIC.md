# Expose FPMS Dashboard to the public internet

This is the "anyone in the world can open it" mode. The dashboard **still
runs on your laptop** — Cloudflare Tunnel is just a free relay that gives
you a public HTTPS URL forwarding to `localhost:8000`.

## What you get

- A URL like `https://random-words-here.trycloudflare.com` your teammates,
  judges, or family can open from anywhere
- Auto HTTPS (Cloudflare provides the cert)
- Works even behind a home NAT — no port forwarding, no static IP
- Kill the tunnel → the URL stops working immediately

## Steps

### 1. Set a strong password (required)

The `Publish-Public.bat` script refuses to run without one. In the same
terminal (or in "Environment Variables" for permanent):

```
set FPMS_PASSWORD=pick-something-long-and-random
```

Everyone who opens the public URL sees the FPMS login screen and needs
that password. Change it and restart the dashboard to rotate access.

### 2. Start the dashboard normally

Double-click `FPMS-Dashboard.exe`. Verify `http://localhost:8000` prompts
for the password.

### 3. Publish

Double-click `Publish-Public.bat`. First run downloads `cloudflared` (~35 MB)
into `bin/`; subsequent runs are instant.

Watch the console — look for the line:

```
Your quick Tunnel has been created! Visit it at:
    https://xxxxx-xxxxx-xxxxx.trycloudflare.com
```

Share that URL. Anyone opening it hits the FPMS login screen; only people
with the password get in.

### 4. Take it offline

`Ctrl+C` in the tunnel window. The URL stops resolving instantly. Doing
this any time you're not actively demoing is a good habit.

## What's protected — and what's not

| Feature | Public visitor access |
|---|---|
| Overview / LiDAR / Camera / Thermal | ✅ (after password) |
| AWS console + Verify | ✅ (after password) |
| Devices — LAN scan + SSH + provision | ✅ (after password) — **they can scan** your network from your laptop |
| Terminal → SSH (to hosts they know creds for) | ✅ (after password) |
| Terminal → Local PowerShell | ❌ Always blocked from non-LAN clients |

If you don't want public visitors doing LAN scans on your network, don't
share the password with anyone you don't trust to that level. Rotate
`FPMS_PASSWORD` any time.

## Upgrade to a stable named tunnel

The `trycloudflare.com` URL rotates on every restart. For a stable URL:

1. `cloudflared login` (opens Cloudflare — takes 60s to set up an account)
2. `cloudflared tunnel create fpms`
3. `cloudflared tunnel route dns fpms fpms.yourdomain.com`
4. Change `Publish-Public.bat` to `cloudflared tunnel run fpms`
