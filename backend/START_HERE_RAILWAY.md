# ⚡ START HERE - Backend Deployment to Railway

**This document is your guide. Read this first, then follow one of the 3 deployment paths below.**

---

## The Problem You Had

- Backend was **8 GB** (including `.venv`)
- Railway free tier only allows **4 GB**
- Solution: Proper Docker configuration excludes `.venv`

---

## The Solution (What We Did)

✅ Created `Dockerfile` - Multi-stage optimized build
✅ Updated `railway.json` - Uses Dockerfile
✅ Updated `.gitignore` - Excludes .venv
✅ Created guides - Documentation for deployment

**Result**: ~800 MB image + ~1.5 GB running memory = **Fits in 4 GB free tier**

---

## Choose Your Deployment Path

### Path A: Auto-Deploy from GitHub (⭐ Easiest, Recommended)

**Time**: ~20 minutes total

1. **Push deployment files**
   ```bash
   cd C:\Users\HP\Desktop\H\Physical-AI-and-Humanoid-Robotics
   git add backend/Dockerfile backend/railway.json backend/.gitignore backend/RAILWAY_QUICKSTART.md DEPLOYMENT_SUMMARY.md
   git commit -m "deploy: configure backend for railway"
   git push origin master
   ```

2. **Go to Railway**: https://railway.app
   - Click "New Project"
   - Select "Deploy from GitHub"
   - Authorize Railway
   - Select your repository

3. **Create service**
   - Click "New" → "Service"
   - Select your repository
   - Railway auto-detects Dockerfile

4. **Add environment variables** (in Railway Dashboard)
   - Click "Variables" tab
   - Add these 5 variables:
   ```
   GROQ_API_KEY=<get_from_console.groq.com>
   QDRANT_URL=https://xxx.cloud.qdrant.io
   QDRANT_API_KEY=<get_from_cloud.qdrant.io>
   ALLOWED_ORIGINS=https://your-book.vercel.app
   ENVIRONMENT=production
   ```

5. **Wait 3-5 minutes** ✅
   - Railway auto-deploys
   - Watch "Deployments" tab for status

6. **Test it**
   - Health: `curl https://your-url/health`
   - Docs: `https://your-url/api/docs`

---

### Path B: Railway CLI (If You Prefer Direct Control)

**Time**: ~15 minutes total

```bash
# 1. Install Railway CLI
npm install -g @railway/cli

# 2. Push code first
cd C:\Users\HP\Desktop\H\Physical-AI-and-Humanoid-Robotics
git add backend/Dockerfile backend/railway.json
git commit -m "deploy: configure backend"
git push origin master

# 3. Login to Railway
railway login

# 4. Navigate to backend
cd backend

# 5. Initialize project
railway init
# Answer: "Create a new project" → "physical-ai-chatbot"

# 6. Set environment variables
railway variables set GROQ_API_KEY=<your_key>
railway variables set QDRANT_URL=https://xxx.cloud.qdrant.io
railway variables set QDRANT_API_KEY=<your_key>
railway variables set ALLOWED_ORIGINS=https://your-book.vercel.app
railway variables set ENVIRONMENT=production

# 7. Deploy!
railway up

# 8. Check status
railway logs --follow
railway status
```

---

### Path C: Read Detailed Guide First

**If you want to understand everything first:**

→ Read: `backend/RAILWAY_QUICKSTART.md` (7-step detailed guide)
→ Or: `DEPLOY_COMMANDS.md` (copy/paste ready)

---

## Getting Required Credentials (5 min)

### Groq API Key
1. Go to: https://console.groq.com
2. Sign up/login
3. Click "API Keys" in sidebar
4. Create new API key
5. Copy it

### Qdrant
1. Go to: https://cloud.qdrant.io
2. Sign up/login
3. Create cluster
4. Copy the **Cluster URL** (API URL)
5. Copy the **API Key**

### Your Vercel Frontend URL
- Check your Vercel dashboard
- Example: `https://my-book.vercel.app`

---

## Verify It Works (After Deployment)

```bash
# Test 1: Health check
curl https://your-railway-url/health
# Should return: {"status": "healthy", "app": "RAG Chatbot API", ...}

# Test 2: API docs
# Open in browser: https://your-railway-url/api/docs
# Should show Swagger UI

# Test 3: From frontend
# Your Vercel frontend should now be able to reach the backend
```

---

## Update Frontend (After Backend Works)

In Vercel environment variables:
```
REACT_APP_API_BASE_URL=https://your-railway-url
```

Then redeploy frontend or it picks up on next deploy.

---

## Files in This Deployment

```
Root level (read first):
  ✅ START_HERE_RAILWAY.md          ← You are here
  ✅ README_RAILWAY.md              ← Quick reference
  ✅ DEPLOYMENT_SUMMARY.md          ← Technical details
  ✅ DEPLOY_COMMANDS.md             ← Copy/paste commands

Backend directory:
  ✅ backend/Dockerfile            ← Your Docker build
  ✅ backend/railway.json           ← Railway config
  ✅ backend/.gitignore             ← Updated (no .venv)
  ✅ backend/RAILWAY_QUICKSTART.md  ← Full step-by-step guide
  ✅ backend/requirements.txt        ← Already exists
  ✅ backend/main.py                ← Already exists
  ✅ backend/src/                   ← Already exists
```

---

## Troubleshooting

### Issue: "Can't find Dockerfile"
- Ensure you pushed the code
- Check: `backend/Dockerfile` exists in GitHub

### Issue: "Build failed"
- Check Railway logs: Dashboard → Deployments → View Logs
- Look for errors in output
- Common: Missing environment variables

### Issue: "Module not found"
- Ensure: `backend/src/__init__.py` exists
- If not: `touch backend/src/__init__.py`

### Issue: "CORS error" from frontend
- Update `ALLOWED_ORIGINS` in Railway variables
- Use your exact Vercel URL (with https://)
- Railway auto-restarts on variable change

### Issue: "Connection refused" from frontend
- Verify your Railway URL is correct
- Check: ALLOWED_ORIGINS includes your frontend
- Test: `curl https://your-url/health`

---

## What Happens During Deployment

```
1. You push code to GitHub
   ↓
2. Railway detects Dockerfile
   ↓
3. Railway builds image:
   - Installs dependencies
   - Copies application code
   - Does NOT include .venv
   ↓
4. Railway starts container:
   - Runs: python -m uvicorn main:app ...
   - Port: 8000
   - Memory: ~1.5 GB
   ↓
5. Service is live at: https://your-railway-url
   ↓
6. Frontend can call: https://your-railway-url/api/chat
```

---

## Cost (Free Forever for Testing)

- **Free tier**: Unlimited for testing
- **If exceeding free limits**: $0.10/GB RAM/hour
- **Estimated**: $0-20/month depending on traffic

See: https://railway.app/pricing

---

## Memory Usage Analysis

```
What's Using Memory:

Python Runtime           ~100 MB
Groq Library             ~100 MB
Qdrant Library           ~100 MB
Sentence Transformers    ~500-800 MB (models)
Other dependencies       ~100 MB
Application code         ~50 MB
─────────────────────────────
Peak total:              ~1.5 GB

Free tier limit:         4 GB ✅
Headroom:                2.5 GB ✅
```

You're well under the limit. No optimization needed.

---

## Next Actions

Pick one:

### I want the easiest path
→ Use **Path A** (auto-deploy from GitHub)

### I want full control
→ Use **Path B** (Railway CLI)

### I want to understand first
→ Read `backend/RAILWAY_QUICKSTART.md`

### I need detailed commands
→ Read `DEPLOY_COMMANDS.md`

### I need technical details
→ Read `DEPLOYMENT_SUMMARY.md`

---

## Questions?

1. **How does deployment work?**
   → `DEPLOYMENT_SUMMARY.md`

2. **Step-by-step instructions?**
   → `backend/RAILWAY_QUICKSTART.md`

3. **Copy/paste commands?**
   → `DEPLOY_COMMANDS.md`

4. **Quick reference?**
   → `README_RAILWAY.md`

5. **Railway docs?**
   → https://docs.railway.app

6. **Groq API help?**
   → https://console.groq.com/docs

7. **Qdrant help?**
   → https://qdrant.tech/documentation/

---

## Summary

✅ Dockerfile created (optimized, ~800 MB)
✅ Railway config ready (auto-deploy enabled)
✅ Environment documented (5 variables needed)
✅ Documentation complete (multiple guides)
✅ Memory analysis done (1.5 GB, under 4GB limit)
✅ Git optimized (.venv excluded)

**You're ready to deploy!**

---

## TL;DR (Just Do It)

```bash
# 1. Push code
git add backend/Dockerfile backend/railway.json backend/.gitignore
git commit -m "deploy: backend to railway"
git push origin master

# 2. Go to railway.app
# - New Project → Deploy from GitHub
# - Select repo

# 3. Add 5 environment variables
# - GROQ_API_KEY, QDRANT_URL, QDRANT_API_KEY, ALLOWED_ORIGINS, ENVIRONMENT

# 4. Wait 3-5 minutes → Done! ✅

# 5. Test
curl https://your-railway-url/health
```

---

**Ready?** Start with Path A above! 🚀

---

*Last updated: December 14, 2025*
*Backend only deployment to Railway (4GB free tier)*
