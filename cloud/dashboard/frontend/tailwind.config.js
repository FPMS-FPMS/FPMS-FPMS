/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        ember: {
          50: "#fff7ed",
          100: "#ffedd5",
          200: "#fed7aa",
          300: "#fdba74",
          400: "#fb923c",
          500: "#f97316",
          600: "#ea580c",
          700: "#c2410c",
          800: "#9a3412",
          900: "#7c2d12",
          950: "#431407",
        },
        ink: {
          950: "#07090d",
          900: "#0b0f16",
          // 850 is the app-chrome surface: header and footer sit one step
          // above the page background so the shell reads as a frame around
          // the content rather than as more content.
          850: "#0e131d",
          800: "#111826",
          700: "#1a2233",
          600: "#242f45",
        },
      },
      fontFamily: {
        sans: ["Inter", "ui-sans-serif", "system-ui", "sans-serif"],
        mono: ["JetBrains Mono", "ui-monospace", "monospace"],
      },
      // A deliberate type scale. The console is dense, so the small end is
      // where most of it lives; the ratio stays tight so nothing shouts
      // except the things that are supposed to (see .btn-hot, MissionStrip).
      fontSize: {
        "2xs": ["0.6875rem", { lineHeight: "1rem", letterSpacing: "0.01em" }],
      },
      boxShadow: {
        glow: "0 0 40px -10px rgba(249, 115, 22, 0.45)",
        // Two-part card shadow: a tight contact shadow plus a soft ambient
        // one. Reads as a raised surface instead of a drop shadow.
        card: "0 1px 2px 0 rgba(0,0,0,0.55), 0 12px 32px -12px rgba(0,0,0,0.75)",
        "card-hover":
          "0 1px 2px 0 rgba(0,0,0,0.55), 0 18px 44px -14px rgba(0,0,0,0.85)",
        chrome: "0 1px 0 0 rgba(255,255,255,0.04), 0 8px 32px -12px rgba(0,0,0,0.9)",
        alarm: "0 0 0 1px rgba(244,63,94,0.5), 0 8px 28px -8px rgba(244,63,94,0.55)",
      },
      keyframes: {
        // Used by the abort/alarm affordances. Kept in the config so the
        // reduced-motion override in index.css can switch it off in one place.
        alarm: {
          "0%, 100%": { boxShadow: "0 0 0 0 rgba(244,63,94,0.55)" },
          "50%": { boxShadow: "0 0 0 6px rgba(244,63,94,0)" },
        },
        "fade-rise": {
          "0%": { opacity: "0", transform: "translateY(4px)" },
          "100%": { opacity: "1", transform: "translateY(0)" },
        },
      },
      animation: {
        alarm: "alarm 1.4s ease-out infinite",
        "fade-rise": "fade-rise 220ms ease-out both",
      },
    },
  },
  plugins: [],
};
