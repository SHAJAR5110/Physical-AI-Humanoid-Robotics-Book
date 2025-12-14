# Deployment Checklist - Backend to Railway (4GB Free Tier)

## Pre-Deployment Checklist

### Files Ready?
- [ ] `backend/Dockerfile` exists (multi-stage optimized)
- [ ] `backend/railway.json` configured
- [ ] `backend/.gitignore` excludes .venv
- [ ] `backend/requirements.txt` valid
- [ ] `backend/main.py` exists
- [ ] `backend/src/` directory complete

### Credentials Collected?
- [ ] **Groq API Key** (from console.groq.com)
- [ ] **Qdrant URL** (from cloud.qdrant.io)
- [ ] **Qdrant API Key** (from cloud.qdrant.io)
- [ ] **Vercel Frontend URL** (your book URL)

### Documentation Ready?
- [ ] `START_HERE_RAILWAY.md` - read entry point
- [ ] `README_RAILWAY.md` - quick reference available
- [ ] `RAILWAY_QUICKSTART.md` - detailed guide ready
- [ ] `DEPLOY_COMMANDS.md` - copy/paste commands ready
- [ ] `DEPLOYMENT_SUMMARY.md` - technical details available

---

## Deployment Process (Choose One Path)

### Path A: Auto-Deploy from GitHub ⭐ (Recommended, 20 min)

**Step 1: Push Code**
- [ ] `git add backend/Dockerfile backend/railway.json`
- [ ] `git add backend/.gitignore backend/RAILWAY_QUICKSTART.md`
- [ ] `git add DEPLOYMENT_SUMMARY.md DEPLOY_COMMANDS.md`
- [ ] `git commit -m "deploy: configure backend for railway"`
- [ ] `git push origin master`

**Step 2: Connect to Railway**
- [ ] Go to railway.app
- [ ] Create account (free)
- [ ] Click "New Project"
- [ ] Select "Deploy from GitHub"
- [ ] Authorize Railway
- [ ] Select your repository

**Step 3: Create Service**
- [ ] Click "New" → "Service"
- [ ] Select your repository
- [ ] Confirm Dockerfile detected

**Step 4: Environment Variables (in Railway Dashboard)**
- [ ] Go to Variables tab
- [ ] Add `GROQ_API_KEY=<your_key>`
- [ ] Add `QDRANT_URL=https://xxx.cloud.qdrant.io`
- [ ] Add `QDRANT_API_KEY=<your_key>`
- [ ] Add `ALLOWED_ORIGINS=https://your-book.vercel.app`
- [ ] Add `ENVIRONMENT=production`
- [ ] Add `LOG_LEVEL=info` (optional)

**Step 5: Deploy**
- [ ] Save variables
- [ ] Watch Deployments tab
- [ ] Wait 3-5 minutes for auto-deploy
- [ ] See "Success" or "Deployed" status

**Step 6: Test**
- [ ] Get service URL from Railway
- [ ] Test: `curl https://your-url/health`
- [ ] Open: `https://your-url/api/docs`
- [ ] Verify API docs loaded

### Path B: Railway CLI (15 min)

- [ ] Install Node.js (if needed)
- [ ] Run: `npm install -g @railway/cli`
- [ ] Run: `railway login`
- [ ] Go to: `backend` directory
- [ ] Run: `railway init`
- [ ] Answer: "Create new project"
- [ ] Name: `physical-ai-chatbot`
- [ ] Env: `production`
- [ ] Set vars: `railway variables set GROQ_API_KEY=...`
- [ ] Set vars: `railway variables set QDRANT_URL=...`
- [ ] Set vars: `railway variables set QDRANT_API_KEY=...`
- [ ] Set vars: `railway variables set ALLOWED_ORIGINS=...`
- [ ] Set vars: `railway variables set ENVIRONMENT=production`
- [ ] Deploy: `railway up`
- [ ] Check: `railway logs --follow`

### Path C: Read First Then Deploy

- [ ] Read: `START_HERE_RAILWAY.md`
- [ ] Read: `backend/RAILWAY_QUICKSTART.md`
- [ ] Then follow Path A or B above

---

## Post-Deployment Verification

### Is Backend Running?
- [ ] Health check passes: `curl https://your-url/health`
- [ ] Response: `{"status": "healthy", "app": "RAG Chatbot API", ...}`

### Can You Access API Docs?
- [ ] Open: `https://your-url/api/docs`
- [ ] See: Swagger UI with endpoints
- [ ] See: `/api/chat` endpoint available

### Is It Accessible?
- [ ] Check Railway logs: No errors in output
- [ ] Check: "Started server process" message
- [ ] Check: No "ImportError" or "ModuleNotFoundError"

### Can Frontend Reach It?
- [ ] Update frontend: `REACT_APP_API_BASE_URL=https://your-url`
- [ ] Test from frontend: Try chat endpoint
- [ ] Check: No CORS errors in browser console

---

## Monitoring

### Daily Tasks
- [ ] Check Railway dashboard monthly
- [ ] Monitor CPU/memory usage
- [ ] Review error logs

### Issues to Watch
- [ ] High CPU usage (> 80%)
- [ ] Memory approaching limit (> 3 GB)
- [ ] Error rate increasing
- [ ] Response times > 5 seconds

### Alert Setup (Optional)
- [ ] Set up Railway alerts
- [ ] Monitor error rate
- [ ] Monitor memory usage
- [ ] Set up Slack notifications (if desired)

---

## Common Issues & Solutions

### Issue: Build Failed
- [ ] Check: Dockerfile exists in backend/
- [ ] Check: requirements.txt is valid
- [ ] Check: No syntax errors in Python files
- [ ] View logs: Railway → Deployments → View Logs

### Issue: Port Binding Error
- [ ] Railway auto-assigns PORT via environment
- [ ] Should be automatic, no action needed

### Issue: Module Not Found
- [ ] Check: `backend/src/__init__.py` exists
- [ ] Check: All imports relative paths correct
- [ ] Check: No circular imports

### Issue: CORS Error from Frontend
- [ ] Get exact Vercel URL
- [ ] Update `ALLOWED_ORIGINS` in Railway variables
- [ ] Include full URL: `https://your-domain.vercel.app`
- [ ] Railway auto-restarts on variable change

### Issue: Qdrant/Groq Connection Failed
- [ ] Verify API keys are correct
- [ ] Test locally with `.env` file first
- [ ] Check: Services aren't rate-limiting
- [ ] View full logs: `railway logs --follow`

---

## Rollback Plan (If Needed)

### Quick Rollback
- [ ] Go to Railway Dashboard
- [ ] Click "Deployments" tab
- [ ] Select previous deployment
- [ ] Click "Redeploy"

### Manual Rollback
- [ ] Go back to previous git commit
- [ ] Push to master
- [ ] Railway auto-redeploys

### Keeping Multiple Versions
- [ ] Create separate Railway projects
- [ ] One for production
- [ ] One for testing
- [ ] Switch frontend URL as needed

---

## Maintenance Tasks

### Weekly
- [ ] Check logs for errors
- [ ] Monitor performance metrics
- [ ] Verify health endpoint still works

### Monthly
- [ ] Review error patterns
- [ ] Check cost/usage metrics
- [ ] Update dependencies if needed

### As Needed
- [ ] Deploy new features: `git push origin master`
- [ ] Fix bugs: `git push origin master`
- [ ] Update environment variables: Railway Dashboard

---

## Success Criteria

✅ All checked: You're done!

### Minimum Requirements
- [ ] Backend deployed on Railway
- [ ] Health endpoint responds
- [ ] API documentation accessible
- [ ] Environment variables set
- [ ] No deployment errors
- [ ] Memory < 4 GB free tier limit
- [ ] CORS working with frontend
- [ ] Chat endpoint accessible

### Nice to Have
- [ ] Logging visible in Railway dashboard
- [ ] Metrics being tracked
- [ ] Alerts configured
- [ ] Monitoring setup
- [ ] Documentation reviewed

---

## Timeline

| Phase | Time | Status |
|-------|------|--------|
| Preparation | 5 min | ✅ |
| Code Push | 2 min | ✅ |
| Railway Setup | 5 min | ✅ |
| Auto-Deploy | 5 min | ✅ |
| Testing | 5 min | ✅ |
| **Total** | **~20 min** | **✅** |

---

## Deployment Status

- [ ] **Initiated**: Date: ________
- [ ] **Code Pushed**: Date: ________
- [ ] **Railway Connected**: Date: ________
- [ ] **Variables Added**: Date: ________
- [ ] **Deployment Started**: Date: ________
- [ ] **Build Complete**: Date: ________
- [ ] **Testing Passed**: Date: ________
- [ ] **Production Live**: Date: ________

**Service URL**: ___________________________________

**Frontend Updated**: [ ] Yes [ ] No

---

## Notes

```
Add any notes or issues encountered:

_________________________________________________________________________

_________________________________________________________________________

_________________________________________________________________________

```

---

## References

- Railway Docs: https://docs.railway.app
- Groq API: https://console.groq.com/docs
- Qdrant: https://qdrant.tech/documentation/
- This Guide: START_HERE_RAILWAY.md

---

**Last Updated**: December 14, 2025
**Status**: Ready for Deployment ✅

---
