# Backend Deployment Guide - Railway (4GB Free Tier)

This folder contains everything needed to deploy the backend independently to Railway.

## Quick Overview

```
backend/
├── 🐳 Dockerfile                    Multi-stage optimized build
├── 🔧 railway.json                  Railway configuration
├── .dockerignore                     Docker build optimization
├── requirements.txt                  Python dependencies
├── main.py                           FastAPI entry point
├── src/                              Application code
│
├── 📚 START_HERE_RAILWAY.md          ⭐ Read this first (3 paths)
├── 🚀 README_RAILWAY.md              Quick reference (TL;DR)
├── 📋 RAILWAY_QUICKSTART.md          Detailed 7-step guide
├── 💻 DEPLOY_COMMANDS.md             Copy/paste commands
├── 📊 DEPLOYMENT_SUMMARY.md          Technical analysis
├── ✅ DEPLOYMENT_CHECKLIST.md        Step-by-step checklist
└── BACKEND_DEPLOYMENT.md             This file
```

## Why This Folder is Self-Contained

✅ **Dockerfile**: Complete Docker build configuration
✅ **railway.json**: Railway deployment configuration
✅ **.dockerignore**: Excludes .venv and large files
✅ **requirements.txt**: All Python dependencies
✅ **main.py + src/**: Application code
✅ **All documentation**: Ready for deployment
✅ **Git-optimized**: .gitignore excludes .venv

## Resources

### Image Size Analysis
- Docker base image: ~160 MB
- Dependencies: ~600 MB
- Application code: ~10 MB
- **Total: ~800 MB** (fits in 4GB free tier)

### Running Memory
- Peak usage: ~1.5 GB (well under 4GB limit)
- No memory optimization needed

## Deployment Options

### Option 1: Auto-Deploy from GitHub (Easiest)
```bash
# Push this folder to GitHub
git add backend/
git commit -m "deploy: backend to railway"
git push origin master

# Then:
# 1. Go to railway.app
# 2. New Project → Deploy from GitHub
# 3. Select your repo
# 4. Add 5 environment variables (see below)
# 5. Wait 3-5 minutes
```

### Option 2: Direct Railway Deploy
```bash
npm install -g @railway/cli
railway login
cd backend
railway init
railway variables set GROQ_API_KEY=your_key
railway variables set QDRANT_URL=your_url
railway variables set QDRANT_API_KEY=your_key
railway variables set ALLOWED_ORIGINS=https://your-frontend.vercel.app
railway variables set ENVIRONMENT=production
railway up
```

## Required Environment Variables

Set these in Railway Dashboard:

```
GROQ_API_KEY                 Your Groq API key (console.groq.com)
QDRANT_URL                   Your Qdrant cluster URL (cloud.qdrant.io)
QDRANT_API_KEY               Your Qdrant API key (cloud.qdrant.io)
ALLOWED_ORIGINS              Your Vercel frontend URL
ENVIRONMENT                  production
LOG_LEVEL                    info (optional)
```

## Testing After Deployment

```bash
# Health check
curl https://your-railway-url/health

# API docs
https://your-railway-url/api/docs

# From frontend
https://your-railway-url/api/chat
```

## Documentation

Choose the file that matches your needs:

| Document | Purpose | Time |
|----------|---------|------|
| **START_HERE_RAILWAY.md** | Choose your deployment path | 2 min |
| **README_RAILWAY.md** | Quick reference & TL;DR | 3 min |
| **RAILWAY_QUICKSTART.md** | Step-by-step detailed guide | 10 min |
| **DEPLOY_COMMANDS.md** | Copy/paste ready commands | 5 min |
| **DEPLOYMENT_CHECKLIST.md** | Tracking checklist | 20 min |
| **DEPLOYMENT_SUMMARY.md** | Technical deep dive | 10 min |

## What's Included

✅ **Dockerfile**
- Multi-stage build (minimal size)
- Health check included
- .venv excluded
- Optimized for Railway

✅ **.dockerignore**
- Excludes .venv (saves 2-3 GB)
- Excludes .git, __pycache__
- Excludes test files & logs
- Minimizes build context

✅ **railway.json**
- Uses Dockerfile builder
- Auto-restart on failure
- No manual configuration needed

✅ **.gitignore**
- Excludes .venv directory
- Standard Python patterns
- IDE files excluded

✅ **requirements.txt**
- All Python dependencies
- Pinned versions
- Ready for pip install

## Next Steps

1. **Read**: `START_HERE_RAILWAY.md` (2 minutes)
2. **Choose**: Path A (easiest) or Path B (CLI)
3. **Collect**: 3 credentials (5 minutes)
4. **Deploy**: Via GitHub or Railway CLI
5. **Test**: Health check endpoint
6. **Verify**: From frontend

## Cost

- **Free tier**: Sufficient for testing/low traffic
- **Estimated**: $0-20/month (high traffic)
- **See**: https://railway.app/pricing

## Support

- Railway: https://docs.railway.app
- Groq: https://console.groq.com/docs
- Qdrant: https://qdrant.tech/documentation/

## Important Notes

✅ **Only this folder** deploys to Railway
✅ **Frontend** is on Vercel (separate)
✅ **4 GB free tier** is sufficient (no optimization needed)
✅ **.venv** is excluded from deployment
✅ **Health checks** included for monitoring
✅ **CORS configured** for frontend integration

---

## TL;DR - Deploy in 4 Steps

```bash
# Step 1: Push code
git add backend/
git commit -m "deploy: backend to railway"
git push origin master

# Step 2: Go to railway.app
# → New Project → Deploy from GitHub

# Step 3: Add 5 environment variables
# → GROQ_API_KEY, QDRANT_URL, QDRANT_API_KEY, ALLOWED_ORIGINS, ENVIRONMENT

# Step 4: Wait & Test
# → 3-5 minutes → curl https://your-url/health ✅
```

---

**Start here**: `START_HERE_RAILWAY.md`

Happy deploying! 🚀
