# 🚀 Backend Deployment to Railway - Quick Reference

## TL;DR (Do This)

### Step 1: Push Code
```bash
git add backend/Dockerfile backend/railway.json backend/.gitignore backend/RAILWAY_QUICKSTART.md DEPLOYMENT_SUMMARY.md DEPLOY_COMMANDS.md
git commit -m "deploy: configure backend for railway"
git push origin master
```

### Step 2: Railway Setup
1. Go to https://railway.app
2. Click "New Project" → "Deploy from GitHub"
3. Authorize & select your repo
4. Click "New" → "Service" → Select repo

### Step 3: Add Variables (in Railway Dashboard)
```
GROQ_API_KEY=your_key_here
QDRANT_URL=https://xxx.cloud.qdrant.io
QDRANT_API_KEY=your_key
ALLOWED_ORIGINS=https://your-book.vercel.app
ENVIRONMENT=production
```

### Step 4: Wait 3-5 minutes ✅
Railway auto-deploys. Check "Deployments" tab.

### Step 5: Test
```bash
curl https://your-railway-url/health
```

---

## What You're Getting

| Component | Size | Status |
|-----------|------|--------|
| Docker Image | ~800 MB | ✅ Optimized |
| Running Memory | ~1.5 GB | ✅ Under 4GB limit |
| Repository Size | Saved 2-3 GB | ✅ .venv excluded |
| Deployment Time | 3-5 min | ✅ Auto-deploy enabled |
| Cost | Free tier | ✅ No charges |

---

## Files Created

```
✅ backend/Dockerfile          → Multi-stage optimized build
✅ backend/railway.json        → Railway config
✅ backend/.gitignore          → Updated (excludes .venv)
✅ backend/RAILWAY_QUICKSTART.md → Detailed 7-step guide
✅ DEPLOYMENT_SUMMARY.md       → Overview & analysis
✅ DEPLOY_COMMANDS.md          → Copy/paste commands
✅ README_RAILWAY.md           → This file
```

---

## Key Features

✨ **Multi-stage Docker Build**
- Minimal base image (python:3.11-slim)
- Only runtime dependencies included
- ~800 MB total image size

✨ **Railway Configuration**
- Auto-detect from Dockerfile
- Auto-restart on failure
- No manual docker-compose needed

✨ **Memory Optimized**
- Fits in 4GB free tier
- No model pruning needed
- Peak usage ~1.5 GB

✨ **Deployment Automation**
- GitHub push → Auto-deploy in 3-5 min
- Or use Railway CLI for direct control

---

## Environment Variables Needed

Get these first:

1. **Groq API Key**: https://console.groq.com
   - Sign up → API Keys → Create new

2. **Qdrant URL & Key**: https://cloud.qdrant.io
   - Create cluster → Copy URL & API key

3. **Your Vercel Frontend URL**: Check Vercel dashboard
   - Example: `https://my-book.vercel.app`

---

## Deployment Methods

### Auto-Deploy (Easiest)
```bash
git push origin master
# → Railway auto-deploys in 3-5 minutes
```

### Railway CLI
```bash
npm install -g @railway/cli
railway login
cd backend
railway init
railway variables set GROQ_API_KEY=key
railway up
```

### Manual (Dashboard)
1. GitHub → Railway → Auto-detect
2. Add variables in dashboard
3. Done!

---

## Testing

### Health Check
```bash
curl https://your-url/health
# {status: healthy, app: RAG Chatbot API}
```

### API Docs
- Browser: `https://your-url/api/docs`
- Should show Swagger UI

### From Frontend
- Frontend at: https://your-book.vercel.app
- Backend at: https://your-railway-url
- Verify ALLOWED_ORIGINS includes frontend URL

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Build fails | Check: Dockerfile exists, requirements.txt valid |
| Port error | Railway auto-assigns PORT env var (normal) |
| CORS error | Update ALLOWED_ORIGINS in Railway variables |
| Module not found | Ensure `backend/src/__init__.py` exists |
| Qdrant/Groq error | Verify API keys in Railway variables |
| Can't find service | Check Deployments tab for build status |

**Check logs**: Railway Dashboard → Deployments → View Logs

---

## Important Notes

### ✅ Memory
- 4GB free tier is **sufficient**
- No optimization needed
- Peak: ~1.5 GB (well under limit)

### ✅ Storage
- .venv excluded (saves 2-3 GB in repo)
- Image: ~800 MB (efficient)
- Code: ~10 MB

### ✅ No Docker Compose
- Railway handles everything
- Just needs: Dockerfile + requirements.txt

### ✅ Frontend Separate
- Book: Vercel (already deployed)
- Backend: Railway (this guide)
- API: Called from frontend

---

## Cost

- **Free tier**: Perfect for testing/low traffic
- **Estimated paid**: $5-20/month (modest traffic)
- **See**: https://railway.app/pricing

---

## Next Steps

1. ✅ Push code with deployment files
2. ✅ Create Railway account (free)
3. ✅ Connect GitHub repo
4. ✅ Add 3 environment variables
5. ✅ Wait for auto-deploy
6. ✅ Test health endpoint
7. ✅ Update frontend API URL
8. ✅ Done!

---

## Detailed Guides

- **Step-by-Step**: `backend/RAILWAY_QUICKSTART.md`
- **Copy/Paste Commands**: `DEPLOY_COMMANDS.md`
- **Technical Details**: `DEPLOYMENT_SUMMARY.md`

---

## Support

- **Railway**: https://docs.railway.app
- **Groq**: https://console.groq.com/docs
- **Qdrant**: https://qdrant.tech/documentation/

---

**Ready?** Start with: `git push origin master` 🚀

Questions? Read the detailed guides in `backend/RAILWAY_QUICKSTART.md`
