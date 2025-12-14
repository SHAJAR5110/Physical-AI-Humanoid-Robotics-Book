# Railway Deployment - Quick Start Guide

**Backend Only Deployment** - Only the backend deploys to Railway. Frontend (book) is already on Vercel.

## Prerequisites

1. **Railway Account**: [railway.app](https://railway.app)
2. **GitHub**: Code pushed to your GitHub repository
3. **API Keys Ready**:
   - Groq API Key (from [console.groq.com](https://console.groq.com))
   - Qdrant URL & API Key (from [cloud.qdrant.io](https://cloud.qdrant.io))
   - Your Vercel frontend URL (for CORS)

---

## Deployment Method 1: Auto-Deploy from GitHub (Recommended)

### Step 1: Connect GitHub
1. Go to [railway.app](https://railway.app)
2. Click "New Project"
3. Select "Deploy from GitHub"
4. Authorize & select your repository

### Step 2: Configure Service
1. In Railway dashboard, click "New" → "Service"
2. Select your GitHub repo
3. Railway auto-detects:
   - `Dockerfile` in backend/ directory
   - `requirements.txt` for dependencies
   - Port 8000 from Dockerfile

### Step 3: Add Environment Variables
Click your service → "Variables" tab → Add these:

```
GROQ_API_KEY=your_groq_key_here
QDRANT_URL=https://xxx.region.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
ALLOWED_ORIGINS=https://your-frontend.vercel.app,http://localhost:3000
ENVIRONMENT=production
LOG_LEVEL=info
PORT=8000
HOST=0.0.0.0
```

### Step 4: Deploy
- Push code to master/main branch:
```bash
git add .
git commit -m "deploy: backend to railway"
git push origin master
```
- Railway auto-deploys in ~3-5 minutes
- Check "Deployments" tab for status

### Step 5: Get Your URL
Once deployed:
1. Click your service
2. Click "Settings"
3. Copy the service URL
4. Update `ALLOWED_ORIGINS` if needed

---

## Deployment Method 2: Railway CLI (Direct)

### Step 1: Install Railway CLI
```bash
npm install -g @railway/cli
```

### Step 2: Login & Initialize
```bash
railway login
cd backend
railway init
```
- Project name: `physical-ai-chatbot`
- Environment: Select default (production)

### Step 3: Set Variables
```bash
railway variables set GROQ_API_KEY=your_key
railway variables set QDRANT_URL=your_url
railway variables set QDRANT_API_KEY=your_key
railway variables set ALLOWED_ORIGINS=https://your-frontend.vercel.app
railway variables set ENVIRONMENT=production
railway variables set LOG_LEVEL=info
```

### Step 4: Deploy
```bash
railway up
```

### Step 5: View Logs & URL
```bash
railway logs
railway status
```

---

## Testing Your Deployment

### 1. Health Check
```bash
curl https://your-railway-url/health
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

### 2. Root Endpoint
```bash
curl https://your-railway-url/
```

### 3. API Documentation
- **Swagger UI**: `https://your-railway-url/api/docs`
- **ReDoc**: `https://your-railway-url/api/redoc`

### 4. Chat Endpoint (from Frontend)
The frontend should be able to reach:
```
https://your-railway-url/api/chat
```

---

## Storage & Memory (4GB Free Tier)

### Image Size Optimization
- **Base Image**: `python:3.11-slim` (160 MB)
- **Dependencies**: ~600 MB
- **Application Code**: ~10 MB
- **Total**: ~800 MB per image

### Running Memory
- **Base**: ~100 MB
- **Loaded Models**: ~500-800 MB
- **Peak Usage**: ~1-1.5 GB

✅ **Fits in 4GB free tier** - No optimization needed

### Cost
- Free tier: Limited usage (excellent for testing)
- If exceeding free tier: $0.10/GB RAM/hour

---

## Important Files for Deployment

```
backend/
├── Dockerfile              ← Multi-stage optimized build
├── railway.json            ← Railway configuration
├── requirements.txt        ← Python dependencies
├── main.py                 ← FastAPI entry point
├── .gitignore              ← Excludes .venv (saves 2-3 GB in git)
├── .env.example            ← Template for variables
└── src/
    ├── config.py           ← Settings from environment variables
    ├── routers/
    │   └── chat.py         ← Chat endpoints
    └── ...
```

---

## Environment Variables Reference

| Variable | Purpose | Example |
|----------|---------|---------|
| `GROQ_API_KEY` | LLM API authentication | `gsk_...` |
| `QDRANT_URL` | Vector database URL | `https://xxx.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Vector database API key | `your_key` |
| `ALLOWED_ORIGINS` | CORS origins (comma-separated) | `https://frontend.vercel.app` |
| `ENVIRONMENT` | deployment environment | `production` |
| `LOG_LEVEL` | logging verbosity | `info`, `debug`, `warning` |
| `PORT` | application port | `8000` |
| `HOST` | bind address | `0.0.0.0` |

---

## Troubleshooting

### Issue: Build fails
- **Check**: Dockerfile exists in backend/
- **Check**: requirements.txt exists
- **Check**: No circular imports in main.py
- **View logs**: Click Deployment → View Logs

### Issue: Port binding error
- Railway auto-assigns PORT env var
- Dockerfile uses `--port $PORT` from environment

### Issue: Module import errors
- Verify `src/` directory exists
- Check `src/__init__.py` exists
- Verify relative imports are correct

### Issue: CORS errors from frontend
- Get your Vercel URL (e.g., `https://book.vercel.app`)
- Update `ALLOWED_ORIGINS` variable
- Railway auto-restarts on variable change

### Issue: Qdrant/Groq connection fails
1. Verify API keys are correct
2. Test locally first with `.env` file
3. Check if services are rate-limiting
4. View logs: Click Deployment → View Logs

---

## Monitoring & Logs

### View Logs in Dashboard
1. Click your service
2. Click "Deployments" tab
3. Select latest deployment
4. View logs in real-time

### View Logs via CLI
```bash
railway logs --follow
```

### Metrics
- CPU usage
- Memory usage
- Network I/O
- Deployment history

---

## Updates & Redeployment

### Via GitHub (Auto)
```bash
git add .
git commit -m "fix: update endpoint"
git push origin master
```
Railway auto-redeploys ~3-5 minutes later.

### Via CLI
```bash
cd backend
railway up
```

### Via Dashboard
1. Click service
2. Click "Deployments"
3. Click "Trigger Deploy"

---

## Next Steps

After deployment:

1. **Update Frontend**
   - Set API base URL in Vercel environment
   - Example: `REACT_APP_API_BASE_URL=https://your-railway-url`

2. **Monitor**
   - Check logs for errors
   - Monitor CPU/memory usage
   - Set up alerts if needed

3. **Test**
   - Test health endpoint
   - Test from frontend
   - Check API docs

---

## Support

- **Railway Docs**: https://docs.railway.app
- **Railway Community**: https://railway.app/community
- **Groq API**: https://console.groq.com/docs
- **Qdrant**: https://qdrant.tech/documentation/

---

## Cost Estimate

- **Free tier**: Sufficient for testing & low traffic
- **Estimated monthly** (if exceeding free): $5-20 depending on traffic
- **See**: [railway.app/pricing](https://railway.app/pricing)

---

**Deployment complete!** Your backend is now running on Railway and accessible at your service URL.
