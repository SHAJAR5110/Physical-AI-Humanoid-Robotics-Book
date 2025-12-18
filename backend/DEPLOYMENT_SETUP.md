# Backend Deployment Setup

## Quick Start - Production Deployment

This guide walks you through deploying the RAG backend alongside your Vercel frontend.

### Step 1: Prepare Your Environment

1. **Copy environment template**:
```bash
cp .env.example .env
```

2. **Fill in your credentials** in `.env`:
```env
COHERE_API_KEY=sk-your-actual-key-here
QDRANT_URL=https://your-qdrant-instance.io
QDRANT_API_KEY=your-qdrant-api-key
ENV=production
FRONTEND_URL=https://your-vercel-app.vercel.app
```

### Step 2: Install Dependencies

```bash
pip install -r requirements.txt
```

### Step 3: Test Locally

```bash
# Start the server
python api.py
# Or
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

Test it:
```bash
curl http://localhost:8000/health
```

### Step 4: Deploy to Production

Choose one of these options:

#### Option A: Railway (Easiest)

1. Go to [railway.app](https://railway.app)
2. Click "New Project" → "Deploy from GitHub"
3. Select your repository
4. Railway automatically detects `requirements.txt`
5. Add environment variables in Railway dashboard:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `ENV=production`

6. Deploy → Copy your Railway URL

#### Option B: Render

1. Go to [render.com](https://render.com)
2. Create new Web Service
3. Connect your GitHub repository
4. Settings:
   - Name: `rag-backend`
   - Runtime: Python 3
   - Build: `pip install -r requirements.txt`
   - Start: `uvicorn api:app --host 0.0.0.0 --port $PORT`
5. Add environment variables in dashboard
6. Create Service → Copy your Render URL

#### Option C: Heroku

1. Install Heroku CLI
2. Login: `heroku login`
3. Create app: `heroku create your-app-name`
4. Set environment variables:
```bash
heroku config:set COHERE_API_KEY=sk-xxx
heroku config:set QDRANT_URL=https://xxx
heroku config:set QDRANT_API_KEY=xxx
```

5. Deploy: `git push heroku main`

### Step 5: Update Frontend

1. Go to your Vercel project settings
2. Add environment variable:
   - Key: `REACT_APP_API_BASE_URL`
   - Value: `https://your-railway-url` (or Render/Heroku URL)

3. Redeploy Vercel project

### Step 6: Verify Integration

1. Visit your Vercel frontend
2. Open ChatBot widget (💬 button)
3. Ask a question
4. Verify you get a response with sources

## API Endpoints Available

- `POST /api/chat` - Chat interface (used by frontend)
- `POST /ask` - Raw RAG query
- `GET /health` - Health check

## Environment Variables Reference

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `COHERE_API_KEY` | ✅ | Cohere API key | `sk-xxxx` |
| `QDRANT_URL` | ✅ | Qdrant URL | `https://xxx.io` |
| `QDRANT_API_KEY` | ❌ | Qdrant API key (cloud only) | `xxxx` |
| `ENV` | ❌ | Environment (development/production) | `production` |
| `FRONTEND_URL` | ❌ | Frontend URL for CORS | `https://xxx.vercel.app` |
| `SERVER_HOST` | ❌ | Server host | `0.0.0.0` |
| `SERVER_PORT` | ❌ | Server port | `8000` |

## Database Population

After deployment, you need to populate the Qdrant database:

```bash
# Run locally or in your environment
python main.py
```

This:
1. Crawls your Vercel frontend URLs
2. Extracts text from all pages
3. Creates embeddings with Cohere
4. Stores vectors in Qdrant

**Note**: This can take 10-30 minutes depending on site size.

## Monitoring

After deployment:

1. **Check logs**:
   - Railway: Dashboard → Deployments → View Logs
   - Render: Dashboard → Service → Logs
   - Heroku: `heroku logs --tail`

2. **Test health endpoint**:
```bash
curl https://your-backend-url/health
```

3. **Monitor from frontend**:
   - Open browser DevTools → Network tab
   - Ask a question in ChatBot
   - Verify requests go to correct URL
   - Check response format is valid

## Troubleshooting

**"Connection refused" error**:
- Verify backend URL is correct
- Check backend is actually running
- Verify firewall allows incoming requests

**"CORS error" in browser**:
- Check FRONTEND_URL in backend environment
- Verify frontend URL is in `cors_origins` list
- Check Accept-Origin header in response

**"API key invalid" error**:
- Verify Cohere API key is correct
- Check API key in Cohere dashboard

**Slow first response**:
- Normal on first request (cold start)
- Qdrant might be indexing vectors
- Check Cohere API rate limits

## Maintenance

### Regular Tasks

1. **Monitor API usage**:
   - Cohere dashboard for request counts
   - Qdrant dashboard for query counts

2. **Check logs weekly**:
   - Look for errors
   - Monitor response times
   - Track user questions

3. **Update dependencies**:
```bash
pip install --upgrade -r requirements.txt
```

### Scaling

If you get rate-limited:
1. Upgrade Cohere plan
2. Increase Qdrant storage
3. Add caching for common questions
4. Implement request throttling

## Support Resources

- Cohere Docs: https://docs.cohere.com
- Qdrant Docs: https://qdrant.tech/documentation
- FastAPI Docs: https://fastapi.tiangolo.com
- Railway Docs: https://docs.railway.app
- Render Docs: https://render.com/docs
