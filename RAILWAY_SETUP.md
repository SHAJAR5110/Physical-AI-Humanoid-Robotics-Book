# Railway Deployment Setup - Quick Guide

## 🔧 Configuration Files Added

Root-level files to support Railway deployment from backend subdirectory:

- **`railway.json`** - Main Railway configuration
- **`Procfile`** - Alternative deployment format

Backend files:
- **`backend/start.sh`** - Startup script
- **`backend/requirements.txt`** - Python dependencies
- **`backend/railway.json`** - Backend-specific config
- **`backend/RAILWAY_DEPLOYMENT.md`** - Complete deployment guide

---

## 🚀 Deploy to Railway in 5 Minutes

### Step 1: Go to Railway Dashboard
1. Visit [railway.app](https://railway.app)
2. Sign in or create account
3. Click **"Create New Project"**
4. Select **"Deploy from GitHub"**

### Step 2: Connect Repository
1. Authorize Railway to access GitHub
2. Select: `Physical-AI-Humanoid-Robotics-Book`
3. Select branch: `master`
4. Click **"Deploy"**

Railway will automatically:
- Detect the `Procfile` and `railway.json`
- Build the Python environment
- Navigate to `backend/` directory
- Install dependencies from `requirements.txt`
- Run `start.sh` to start FastAPI

### Step 3: Add Environment Variables
In Railway Dashboard:
1. Click your service
2. Go to **"Variables"** tab
3. Add these variables:

```
GROQ_API_KEY=gsk_your_key_here
QDRANT_URL=https://your-cluster.region.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key
ALLOWED_ORIGINS=https://your-frontend.vercel.app,http://localhost:3000
ENVIRONMENT=production
LOG_LEVEL=info
```

### Step 4: Get Your Service URL
Once deployed, Railway provides a public URL:
```
https://physical-ai-chatbot.up.railway.app
```

### Step 5: Test It Works
```bash
curl https://your-service-url/health
```

Expected response:
```json
{
  "status": "healthy",
  "app": "RAG Chatbot API",
  "version": "1.0.0",
  "environment": "production"
}
```

### Step 6: Connect Frontend
Update your Vercel environment variable:
- `REACT_APP_API_BASE_URL=https://your-service-url`

Done! ✅

---

## 🆘 Troubleshooting

### Error: "Script start.sh not found"
**Fixed!** The root-level `railway.json` and `Procfile` now correctly navigate to the `backend/` directory.

### Error: "Module not found"
Check that `backend/src/` directory exists with:
```bash
ls backend/src/
ls backend/src/__init__.py
```

### Can't connect to Groq/Qdrant
1. Verify credentials are correct in Railway Variables
2. Check services aren't rate-limiting
3. View logs: Click your service → Deployments → View logs

### Frontend shows CORS errors
Update `ALLOWED_ORIGINS` variable to include your Vercel frontend URL:
```
ALLOWED_ORIGINS=https://your-domain.vercel.app
```

---

## 📚 Complete Documentation

For detailed information, see **`backend/RAILWAY_DEPLOYMENT.md`**:
- 3 deployment options (CLI, GitHub, manual)
- Full environment variable reference
- Verification procedures
- Advanced troubleshooting
- Performance monitoring

---

## 📋 Deployment Checklist

- [ ] Railway account created
- [ ] Repository connected to Railway
- [ ] Service created and deploying
- [ ] All environment variables added
- [ ] Health endpoint responding (/health)
- [ ] API docs accessible (/api/docs)
- [ ] Frontend connected with correct API URL
- [ ] Testing integration between frontend and backend

---

## 🎉 Success Indicators

Once deployed, you should see:

1. **Green deployment status** in Railway dashboard
2. **Service URL** assigned (e.g., `*.up.railway.app`)
3. **Health check passing**: `/health` returns `{"status": "healthy"}`
4. **API docs available**: `/api/docs` shows Swagger UI
5. **Frontend connecting**: No CORS errors in browser console

You're ready to go! 🚀

