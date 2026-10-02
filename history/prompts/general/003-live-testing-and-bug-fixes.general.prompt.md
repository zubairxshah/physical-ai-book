---
id: 003
title: Live testing and bug fixes
stage: general
date: 2026-10-03
surface: agent
model: claude-opus-5-5
feature: none
branch: main
user: zubairxshah
command: none
labels: ["deployment", "cors", "auth", "translation", "playwright", "security"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - vercel.json
 - src/config/api.ts
 - src/pages/feedback.tsx
 - src/theme/Root.tsx
 - src/utils/translateToUrdu.ts
 - src/components/FloatingToolbar.tsx
 - src/components/EnhancedChapter.tsx
 - huggingface-space/app.py
 - PROJECT_MEMORY.md
tests:
 - Playwright end-to-end test against live site (home, sign-in, Tools panel, tooltip, chatbot, Urdu translation) - all PASS, 0 console errors
 - curl checks of /health, /sign-in (3 test users), /query, /tooltips, /translate
 - Direct handler test of GET /feedback admin-token auth (403/403/200)
---

## Prompt

what we had been doing here?

check if the live deployment still works

can we test it live

try it

https://physical-ai-book-git-fix-same-or-8d860e-m-zubair-shahs-projects.vercel.app

https://vercel.com/m-zubair-shahs-projects/physical-ai-book/physical-ai-book-git-fix-same-or-8d860e-m-zubair-shahs-projects.vercel.app

https://physical-ai-book-xi.vercel.app/

remove the old script from Root.tsx and fix bugs

do this and teach me steps to do them properly

tell me testmail email and password to check login

Things working quite good, let's save progress and we'll improve things later

## Response snapshot

Summarized project state, verified backend with curl, then ran a Playwright browser
test that showed sign-in blocked by CORS (HF Spaces proxy omits
Access-Control-Allow-Credentials on preflight). Fixed by proxying `/api/*` through a
Vercel rewrite (commit bd82310). Then fixed Urdu translation flattening/truncation with a
block-by-block helper, removed the dead chatbot-widget-v2.js loader, locked GET /feedback
behind ADMIN_TOKEN (533c3bf), and enabled pool_pre_ping for the intermittent post-idle
sign-in 500 (b7ee54b). Frontend deployed and verified live; backend changes await manual
upload to the HF Space. Gave the user step-by-step deployment instructions and test logins.

## Outcome

- ✅ Impact: Sign-in, personalization tooltips and Urdu translation work in real browsers on the live site; translation keeps chapter structure and covers full chapters.
- 🧪 Tests: Live Playwright e2e 6/6 PASS with 0 console errors; feedback auth handler test passed.
- 📁 Files: 8 source/config files changed, PROJECT_MEMORY.md updated.
- 🔁 Next prompts: Verify backend after user uploads app.py + ADMIN_TOKEN; preserve inline formatting in translations; refresh stale CLAUDE.md status.
- 🧠 Reflection: curl-only checks missed a browser-only CORS failure; always test cross-origin features in a real browser.

## Evaluation notes (flywheel)

- Failure modes observed: Vercel preview protection blocked automated preview testing; initial test assumed toolbar was always visible (it is a hover dropdown).
- Graders run and results (PASS/FAIL): Playwright e2e PASS (live).
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Configure a Vercel protection-bypass secret so previews can be tested before merging.
