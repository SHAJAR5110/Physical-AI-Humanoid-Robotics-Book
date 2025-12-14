# Railway Backend Deployment - Ready-to-Copy Commands

## Method 1: GitHub Auto-Deploy (Easiest - Recommended)

### Step 1: Push Code to GitHub
```bash
cd C:\Users\HP\Desktop\H\Physical-AI-and-Humanoid-Robotics
git add backend/Dockerfile backend/railway.json backend/.gitignore backend/RAILWAY_QUICKSTART.md DEPLOYMENT_SUMMARY.md DEPLOY_COMMANDS.md
git commit -m "deploy: configure backend for railway deployment"
git push origin master
```

### Step 2: Connect Railway Dashboard
1. Go to https://railway.app
2. Click "New Project"
3. Select "Deploy from GitHub"
4. Authorize Railway & select your repo
5. Click "New" → "Service" → Select your repo

### Step 3: Add Environment Variables in Railway Dashboard

Railway Dashboard → Your Service → Variables Tab

```
GROQ_API_KEY=<your_groq_api_key_here>
QDRANT_URL=https://xxx.region.cloud.qdrant.io
QDRANT_API_KEY=<your_qdrant_api_key_here>
ALLOWED_ORIGINS=https://your-book.vercel.app,http://localhost:3000
ENVIRONMENT=production
LOG_LEVEL=info
PORT=8000
HOST=0.0.0.0
```

### Step 4: Done ✅
Railway auto-deploys in 3-5 minutes. Watch "Deployments" tab for status.

---

## Method 2: Railway CLI (If Preferred)

### Step 1: Install Railway CLI
```bash
npm install -g @railway/cli
```

### Step 2: Push Code First
```bash
cd C:\Users\HP\Desktop\H\Physical-AI-and-Humanoid-Robotics
git add backend/Dockerfile backend/railway.json backend/.gitignore backend/RAILWAY_QUICKSTART.md DEPLOYMENT_SUMMARY.md DEPLOY_COMMANDS.md
git commit -m "deploy: configure backend for railway deployment"
git push origin master
```

### Step 3: Login & Initialize Railway Project
```bash
railway login
cd backend
railway init
# Select: "Create a new project"
# Project name: physical-ai-chatbot
# Environment: default (production)
```

### Step 4: Set Environment Variables
```bash
railway variables set GROQ_API_KEY=<your_groq_api_key>
railway variables set QDRANT_URL=https://xxx.region.cloud.qdrant.io
railway variables set QDRANT_API_KEY=<your_qdrant_api_key>
railway variables set ALLOWED_ORIGINS=https://your-book.vercel.app,http://localhost:3000
railway variables set ENVIRONMENT=production
railway variables set LOG_LEVEL=info
railway variables set PORT=8000
railway variables set HOST=0.0.0.0
```

### Step 5: Deploy
```bash
railway up
```

### Step 6: View Status
```bash
railway status
railway logs
```

---

## Getting Your Credentials

### Get Groq API Key
1. Go to https://console.groq.com
2. Sign in (create account if needed)
3. Click "API Keys" in sidebar
4. Click "Create New API Key"
5. Copy the key

### Get Qdrant URL & Key
1. Go to https://cloud.qdrant.io
2. Sign in (create account if needed)
3. Click your cluster
4. Click "Overview"
5. Copy the Cluster URL (API URL)
6. Click "Show API Key"
7. Copy the API key

### Get Your Vercel Frontend URL
- Check your Vercel dashboard
- Example: `https://your-project.vercel.app`

---

## Verify Deployment Works

### Test 1: Health Check
```bash
curl https://your-railway-service-url/health
```

Expected:
```json
{
  "status": "healthy",
  "app": "RAG Chatbot API",
  "version": "1.0.0",
  "environment": "production"
}
```

### Test 2: Root Endpoint
```bash
curl https://your-railway-service-url/
```

### Test 3: API Documentation
- Open in browser: `https://your-railway-service-url/api/docs`
- Should show Swagger UI with all endpoints

### Test 4: From Frontend
- Frontend should be able to reach `/api/chat` endpoint
- Check browser console for CORS issues

---

## Useful Railway Commands

```bash
# View logs (live)
railway logs --follow

# Check status
railway status

# Redeploy latest code
railway up

# View variables
railway variables list

# Update a variable
railway variables set VAR_NAME=new_value

# Remove a variable
railway variables delete VAR_NAME

# View service info
railway service list

# Open dashboard in browser
railway open
```

---

## If Deployment Fails

### Check Logs
```bash
railway logs --follow
```

Look for:
- Python syntax errors
- Module import errors (missing `src/__init__.py`?)
- Environment variable issues

### Common Issues

**Issue**: "Module not found: src"
- **Fix**: Ensure `backend/src/__init__.py` exists
```bash
touch backend/src/__init__.py
```

**Issue**: "Port already in use"
- **Fix**: Railway auto-assigns PORT env var, should be fine

**Issue**: "GROQ_API_KEY" or "QDRANT_URL" errors
- **Fix**: Verify variables in Railway dashboard are correct
- **Fix**: Verify API key is valid (test locally first)

**Issue**: CORS errors from frontend
- **Fix**: Update `ALLOWED_ORIGINS` to match your Vercel URL exactly
- **Fix**: Include trailing slash if needed

### Debug Locally First
```bash
cd backend
cp .env.example .env
# Edit .env with your real API keys
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Then test: `curl http://localhost:8000/health`

---

## After Deployment

### Update Frontend Environment
In Vercel dashboard or `.env.local`:
```
REACT_APP_API_BASE_URL=https://your-railway-service-url
```

### Monitor
- Check Railway dashboard regularly
- Watch CPU/memory usage
- Review logs for errors
- Set up Slack alerts (optional)

### Updates
Push code changes and Railway auto-deploys:
```bash
git add .
git commit -m "fix: update endpoint"
git push origin master
```

---

## File Checklist

✅ `backend/Dockerfile` - Multi-stage optimized build
✅ `backend/railway.json` - Railway configuration
✅ `backend/.gitignore` - Excludes .venv
✅ `backend/RAILWAY_QUICKSTART.md` - Detailed guide
✅ `backend/requirements.txt` - Dependencies
✅ `backend/main.py` - FastAPI app
✅ `backend/src/` - Application code
✅ `DEPLOYMENT_SUMMARY.md` - Overview
✅ `DEPLOY_COMMANDS.md` - This file

---

## Summary

1. **Push code** with deployment files
2. **Connect GitHub to Railway** (or use CLI)
3. **Add environment variables** in Railway
4. **Wait 3-5 minutes** for auto-deploy
5. **Test health endpoint** to verify
6. **Update frontend** with API URL
7. **Done!** Backend running on Railway

---

**Need help?**
- Railway Docs: https://docs.railway.app
- Check logs: `railway logs --follow`
- Read: `backend/RAILWAY_QUICKSTART.md`

**Ready to deploy?** Start with the commands above! 🚀
