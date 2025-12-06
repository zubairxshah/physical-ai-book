# 🚀 Deployment Checklist

Use this checklist to ensure everything is properly deployed for the hackathon.

## Pre-Deployment

### API Keys Setup
- [ ] OpenAI API key obtained
- [ ] Qdrant Cloud account created
- [ ] Qdrant cluster created (free tier)
- [ ] All keys saved securely
- [ ] `.env` file created (NOT committed to git)

### Local Testing
- [ ] Backend runs locally (`uvicorn main:app --reload`)
- [ ] `/health` endpoint works
- [ ] `/ingest` completed successfully
- [ ] `/query` returns answers
- [ ] Frontend runs locally (`npm start`)
- [ ] Chatbot widget appears
- [ ] Chatbot responds to questions
- [ ] Text selection works
- [ ] Sources are displayed

## Backend Deployment (Render.com)

### Account Setup
- [ ] Render.com account created
- [ ] GitHub repository connected
- [ ] Payment method added (for verification, free tier available)

### Service Configuration
- [ ] New Web Service created
- [ ] Repository selected
- [ ] Root directory set to `backend`
- [ ] Build command: `pip install -r requirements.txt`
- [ ] Start command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
- [ ] Environment variables added:
  - [ ] `OPENAI_API_KEY`
  - [ ] `QDRANT_URL`
  - [ ] `QDRANT_API_KEY`

### Deployment Verification
- [ ] Service deployed successfully
- [ ] No build errors
- [ ] Service is "Live"
- [ ] Health check passes: `https://your-app.onrender.com/health`
- [ ] API docs accessible: `https://your-app.onrender.com/docs`

### Content Ingestion
- [ ] Run: `curl -X POST https://your-app.onrender.com/ingest`
- [ ] Verify success message
- [ ] Check Qdrant dashboard for vectors
- [ ] Test query: `curl -X POST https://your-app.onrender.com/query -H "Content-Type: application/json" -d '{"question":"What is Physical AI?"}'`

## Frontend Deployment (GitHub Pages)

### Repository Setup
- [ ] GitHub repository created
- [ ] Repository is public
- [ ] All code pushed to main branch
- [ ] `.gitignore` configured (no `.env` files)

### Docusaurus Configuration
- [ ] `docusaurus.config.ts` updated:
  - [ ] `url`: `https://YOUR_USERNAME.github.io`
  - [ ] `baseUrl`: `/physical-ai-book/`
  - [ ] `organizationName`: `YOUR_USERNAME`
  - [ ] `projectName`: `physical-ai-book`

### Chatbot Widget Update
- [ ] `static/chatbot-widget.js` updated:
  - [ ] `API_URL` changed to Render URL
  - [ ] No `localhost` references remain

### Build and Deploy
- [ ] Run: `npm install`
- [ ] Run: `npm run build` (no errors)
- [ ] Run: `npm run deploy`
- [ ] Wait for deployment to complete

### GitHub Pages Configuration
- [ ] Go to repository Settings → Pages
- [ ] Source: Deploy from branch
- [ ] Branch: `gh-pages` / root
- [ ] Save and wait for deployment

### Deployment Verification
- [ ] Site accessible at `https://YOUR_USERNAME.github.io/physical-ai-book/`
- [ ] All pages load correctly
- [ ] Navigation works
- [ ] Chatbot widget appears (💬 button)
- [ ] Chatbot connects to backend
- [ ] Chatbot responds to questions
- [ ] Text selection feature works
- [ ] Mobile view works

## Testing

### Functional Testing
- [ ] Test on Chrome
- [ ] Test on Firefox
- [ ] Test on Safari (if available)
- [ ] Test on mobile device
- [ ] Test all navigation links
- [ ] Test chatbot with 5+ different questions
- [ ] Test text selection feature
- [ ] Verify source citations appear

### Performance Testing
- [ ] Page load time < 3 seconds
- [ ] Chatbot response time < 5 seconds
- [ ] No console errors
- [ ] No broken links
- [ ] Images load correctly

### Error Handling
- [ ] Test with invalid question
- [ ] Test with very long question
- [ ] Test with special characters
- [ ] Test with empty input
- [ ] Verify error messages are user-friendly

## Documentation

### README Files
- [ ] Main `README.md` complete
- [ ] `DEPLOYMENT.md` accurate
- [ ] `QUICKSTART.md` tested
- [ ] `backend/README.md` complete
- [ ] All URLs updated (no localhost)

### Code Quality
- [ ] Code commented
- [ ] No sensitive data in code
- [ ] No console.log in production
- [ ] Proper error handling
- [ ] Clean git history

## Demo Video

### Recording
- [ ] Screen recording software ready
- [ ] Audio quality checked
- [ ] Script prepared
- [ ] Demo environment clean

### Content (5-7 minutes)
- [ ] Introduction (30s)
- [ ] Book navigation (1m)
- [ ] General chatbot queries (1m)
- [ ] Text selection demo (1m)
- [ ] Source citations (30s)
- [ ] Mobile view (30s)
- [ ] Code walkthrough (1-2m)
- [ ] Architecture explanation (1m)

### Upload
- [ ] Video edited (if needed)
- [ ] Uploaded to YouTube/Loom
- [ ] Set to Public/Unlisted
- [ ] Link copied
- [ ] Link tested (opens correctly)

## Submission

### Form Preparation
- [ ] Project name ready
- [ ] GitHub repository URL
- [ ] Live website URL
- [ ] API backend URL
- [ ] Demo video URL
- [ ] Description written
- [ ] Screenshots prepared

### Final Checks
- [ ] All URLs work
- [ ] Demo video plays
- [ ] Repository is public
- [ ] README is complete
- [ ] No broken links
- [ ] Everything tested one last time

### Submit
- [ ] Form filled: https://forms.gle/rw7Lepqwbob1y6hf6
- [ ] Confirmation received
- [ ] Zoom link saved for Monday 6 PM

## Post-Submission

### Monitoring
- [ ] Check Render logs for errors
- [ ] Monitor OpenAI usage
- [ ] Check Qdrant storage
- [ ] Test site periodically

### Backup
- [ ] Code backed up locally
- [ ] Environment variables saved securely
- [ ] Screenshots saved
- [ ] Demo video downloaded

### Presentation Prep
- [ ] Prepare 2-minute pitch
- [ ] Practice demo
- [ ] Prepare for questions
- [ ] Test everything before Zoom call

## Common Issues & Solutions

### Issue: Chatbot not responding
**Solution**: 
- Check browser console for errors
- Verify API_URL in chatbot-widget.js
- Check Render service is running
- Test backend directly with curl

### Issue: CORS errors
**Solution**:
- Verify CORS middleware in main.py
- Check allow_origins includes your GitHub Pages URL
- Restart Render service

### Issue: "Collection not found"
**Solution**:
- Run /ingest endpoint again
- Check Qdrant dashboard
- Verify environment variables

### Issue: Build fails on Render
**Solution**:
- Check requirements.txt
- Verify Python version
- Check Render logs for specific error
- Ensure all dependencies are listed

### Issue: GitHub Pages 404
**Solution**:
- Check baseUrl in docusaurus.config.ts
- Verify gh-pages branch exists
- Wait 5-10 minutes for propagation
- Check GitHub Actions for errors

## Emergency Contacts

- **GIAIC Support**: [Contact info]
- **Render Support**: https://render.com/docs
- **Qdrant Support**: https://qdrant.tech/documentation/
- **OpenAI Support**: https://help.openai.com/

---

## ✅ Final Verification

Before submitting, verify ALL of these:

1. ✅ Live website loads: `https://YOUR_USERNAME.github.io/physical-ai-book/`
2. ✅ Backend health check: `https://your-app.onrender.com/health`
3. ✅ Chatbot responds to: "What is Physical AI?"
4. ✅ Text selection works
5. ✅ Demo video plays
6. ✅ README has correct URLs
7. ✅ Repository is public
8. ✅ No sensitive data committed

---

**When all boxes are checked, you're ready to submit! 🎉**

Good luck with your hackathon! 🚀
