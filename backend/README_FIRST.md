# 🚀 Deploy Backend to Railway - Start Here

**Backend Folder is now fully self-contained. Deploy only the `backend/` folder.**

---

## Quick Start (Choose One)

### Option 1: Auto-Deploy from GitHub (⭐ Easiest, 20 min)

```bash
# 1. Push backend folder to GitHub
cd backend
git add .
git commit -m "deploy: backend to railway"
git push origin master

# 2. Go to: https://railway.app
#    • New Project → Deploy from GitHub
#    • Select your repository
#    • Railway auto-detects Dockerfile

# 3. Add 5 Environment Variables:
#    GROQ_API_KEY=<get_from_console.groq.com>
#    QDRANT_URL=https://xxx.cloud.qdrant.io
#    QDRANT_API_KEY=<get_from_cloud.qdrant.io>
#    ALLOWED_ORIGINS=https://your-book.vercel.app
#    ENVIRONMENT=production

# 4. Done! Railway auto-deploys in 3-5 minutes ✅
```

### Option 2: Railway CLI (15 min)

```bash
npm install -g @railway/cli
railway login
cd backend
railway init
# Select: "Create new project" → "physical-ai-chatbot"

# Set variables
railway variables set GROQ_API_KEY=your_key
railway variables set QDRANT_URL=https://xxx.cloud.qdrant.io
railway variables set QDRANT_API_KEY=your_key
railway variables set ALLOWED_ORIGINS=https://your-book.vercel.app
railway variables set ENVIRONMENT=production

# Deploy!
railway up
```

---

## What's in This Folder

```
backend/
├── 🐳 Dockerfile              Multi-stage optimized (~800 MB)
├── 🔧 railway.json            Railway configuration
├── .dockerignore              Docker build optimization
├── requirements.txt           Python dependencies
├── main.py                    FastAPI entry point
├── src/                       Application code
│
├── 📖 README_FIRST.md         This file (START HERE!)
├── 📌 START_HERE_RAILWAY.md   Detailed guide (3 paths)
├── 🚀 README_RAILWAY.md       Quick TL;DR reference
├── 📋 DEPLOY_COMMANDS.md      Copy/paste commands
├── ✅ DEPLOYMENT_CHECKLIST.md Step-by-step checklist
├── 📊 DEPLOYMENT_SUMMARY.md   Technical analysis
├── 📚 RAILWAY_QUICKSTART.md   7-step detailed guide
└── BACKEND_DEPLOYMENT.md      Backend overview
```

---

## Required Credentials (5 minutes to collect)

1. **Groq API Key**
   - Go to: https://console.groq.com
   - Sign up/login
   - API Keys → Create new
   - Copy the key

2. **Qdrant URL & API Key**
   - Go to: https://cloud.qdrant.io
   - Sign up/login
   - Create cluster
   - Copy URL and API key

3. **Vercel Frontend URL**
   - Check your Vercel dashboard
   - Example: `https://my-book.vercel.app`

---

## After Deployment

### Test 1: Health Check
```bash
curl https://your-railway-url/health
# Response: {"status": "healthy", "app": "RAG Chatbot API", ...}
```

### Test 2: API Docs
- Browser: `https://your-railway-url/api/docs`
- Should show Swagger UI

### Test 3: Update Frontend
In Vercel environment variables:
```
REACT_APP_API_BASE_URL=https://your-railway-url
```

---

## Why This Works

✅ **Dockerfile**: Multi-stage optimized build
✅ **.dockerignore**: Excludes .venv (saves 2-3 GB)
✅ **railway.json**: Auto-detects Dockerfile
✅ **requirements.txt**: All dependencies
✅ **4 GB free tier**: Sufficient (uses ~1.5 GB)
✅ **Self-contained**: Just deploy this folder

---

## Documentation Guide

| Want to... | Read... | Time |
|-----------|---------|------|
| Understand options | START_HERE_RAILWAY.md | 5 min |
| Quick reference | README_RAILWAY.md | 3 min |
| Step-by-step | RAILWAY_QUICKSTART.md | 10 min |
| Copy/paste commands | DEPLOY_COMMANDS.md | 5 min |
| Track progress | DEPLOYMENT_CHECKLIST.md | 20 min |
| Technical details | DEPLOYMENT_SUMMARY.md | 10 min |

---

## Size Analysis

```
Docker Image:          ~800 MB  ✅
Peak Runtime Memory:   ~1.5 GB  ✅
Free Tier Limit:       4 GB     ✅ SUFFICIENT
Repository Savings:    2-3 GB   ✅ (.venv excluded)
```

---

## Key Files

**For Deployment:**
- `Dockerfile` - How to build the image
- `railway.json` - Railway configuration
- `.dockerignore` - What to exclude from build
- `requirements.txt` - Python packages

**For Your App:**
- `main.py` - FastAPI application
- `src/` - Application code

**For Documentation:**
- `START_HERE_RAILWAY.md` - 3 deployment paths
- `DEPLOY_COMMANDS.md` - Copy/paste ready
- `RAILWAY_QUICKSTART.md` - Detailed guide

---

## Troubleshooting

**Build failed?**
- Check: Dockerfile exists
- Check: requirements.txt is valid
- View: Railway Dashboard → Deployments → Logs

**CORS error from frontend?**
- Update: ALLOWED_ORIGINS in Railway variables
- Use: Your exact Vercel URL
- Wait: Railway auto-restarts on variable change

**Connection refused?**
- Verify: GROQ_API_KEY and QDRANT credentials
- Test: `curl https://your-url/health`
- Check: Railway logs for errors

---

## Next Steps

1. ✅ Choose deployment option (A or B above)
2. ✅ Collect 3 credentials (5 min)
3. ✅ Deploy via GitHub or Railway CLI
4. ✅ Wait 3-5 minutes
5. ✅ Test health endpoint
6. ✅ Update frontend URL
7. ✅ Done!

---

## Cost

- **Free tier**: Perfect for testing/low traffic
- **Estimated**: $0-20/month (high traffic)
- Details: https://railway.app/pricing

---

## Support

- Railway: https://docs.railway.app
- Groq: https://console.groq.com/docs
- Qdrant: https://qdrant.tech/documentation/

---

## TL;DR

```bash
# Deploy this folder only
git add backend/
git commit -m "deploy: backend to railway"
git push origin master

# Go to railway.app → New Project → Deploy from GitHub
# Add 5 environment variables → Wait 3-5 min → Done! ✅
```

---

**Ready?** Choose Option 1 or 2 above and start deploying! 🚀

For detailed guide, read: `START_HERE_RAILWAY.md`

