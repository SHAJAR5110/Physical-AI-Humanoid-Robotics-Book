# Railway Deployment Guide - RAG Chatbot Backend

This guide walks you through deploying the FastAPI backend to Railway.com.

## Prerequisites

1. **Railway Account**: Sign up at [railway.app](https://railway.app)
2. **GitHub Repository**: Your code should be in a public GitHub repository (or private with Railway access granted)
3. **Environment Variables Ready**: Have your Groq API key and Qdrant credentials prepared

## Deployment Steps

### Option 1: Deploy via Railway CLI (Recommended)

#### Step 1: Install Railway CLI
```bash
npm install -g @railway/cli
```

#### Step 2: Login to Railway
```bash
railway login
```

#### Step 3: Navigate to Backend Directory
```bash
cd backend
```

#### Step 4: Initialize Railway Project
```bash
railway init
```
- Select: Create a new project
- Project name: `physical-ai-chatbot` (or your choice)
- Environment: Select default (production)

#### Step 5: Add Environment Variables
```bash
railway variables set GROQ_API_KEY=your_groq_key
railway variables set QDRANT_URL=your_qdrant_url
railway variables set QDRANT_API_KEY=your_qdrant_key
railway variables set ALLOWED_ORIGINS=https://your-frontend-url,http://localhost:3000
railway variables set ENVIRONMENT=production
railway variables set LOG_LEVEL=info
railway variables set PORT=8000
```

Or set them via Railway Dashboard:
1. Go to your project dashboard
2. Click "Variables" tab
3. Add each variable from `.env.example`

#### Step 6: Deploy
```bash
railway up
```

Railway will:
- Build the Docker image from your code
- Install dependencies from `requirements.txt`
- Run `start.sh` to start the server
- Assign a public URL to your service

#### Step 7: Get Your Service URL
```bash
railway link
```
This displays your deployed URL (e.g., `https://physical-ai-chatbot.up.railway.app`)

### Option 2: Deploy via GitHub (Push-to-Deploy)

#### Step 1: Connect GitHub Repository
1. Go to [railway.app](https://railway.app)
2. Create a new project
3. Select "Deploy from GitHub"
4. Authorize Railway to access your GitHub account
5. Select your repository

#### Step 2: Configure Service
1. In Railway Dashboard → Select your project
2. Click "New" → "Service"
3. Choose "GitHub Repo" and select your repository
4. Railway auto-detects the `start.sh` and `requirements.txt`

#### Step 3: Add Environment Variables
In Railway Dashboard:
1. Click your service
2. Go to "Variables" tab
3. Add each variable from `.env.example`:
   - `GROQ_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `ALLOWED_ORIGINS` (include your Vercel frontend URL)
   - `ENVIRONMENT=production`
   - `LOG_LEVEL=info`

#### Step 4: Deploy
- Railway auto-deploys when you push to `main`/`master` branch
- Or manually trigger deployment in dashboard

### Option 3: Manual Dockerfile (Advanced)

If you want more control, create a `Dockerfile` in the backend directory:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8000/health')"

# Run application
CMD ["bash", "start.sh"]
```

Then push this Dockerfile to your repository and Railway will auto-detect it.

---

## Environment Variables Configuration

Railway needs these variables for the backend to function:

| Variable | Value | Example |
|----------|-------|---------|
| `GROQ_API_KEY` | Your Groq API key | `gsk_...` |
| `QDRANT_URL` | Qdrant Cloud cluster URL | `https://xxx.region.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `xxxxxx` |
| `ALLOWED_ORIGINS` | CORS allowed origins (comma-separated) | `https://yourdomain.vercel.app,http://localhost:3000` |
| `ENVIRONMENT` | deployment environment | `production` |
| `LOG_LEVEL` | logging level | `info` |
| `PORT` | server port (Railway auto-assigns) | `8000` |
| `HOST` | server host | `0.0.0.0` |

### Getting Credentials

#### Groq API Key
1. Go to [console.groq.com](https://console.groq.com)
2. Sign in with your account
3. Navigate to "API Keys"
4. Create a new API key
5. Copy the key and add to Railway variables

#### Qdrant Credentials
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Sign in
3. Create or select your cluster
4. Click "Show API Key"
5. Copy URL and API key to Railway variables

---

## Verification & Testing

### 1. Check Deployment Status
```bash
railway status
```

### 2. View Logs
```bash
railway logs
```

Or in Railway Dashboard:
1. Click your service
2. Click "Deployments" tab
3. Select latest deployment
4. View logs in real-time

### 3. Test Health Endpoint
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

### 4. Test Root Endpoint
```bash
curl https://your-service-url/
```

Expected response:
```json
{
  "message": "RAG Chatbot API",
  "docs": "/api/docs",
  "health": "/health"
}
```

### 5. Access API Documentation
- **Swagger UI**: `https://your-service-url/api/docs`
- **ReDoc**: `https://your-service-url/api/redoc`

---

## Troubleshooting

### Issue: Build fails with "requirements.txt not found"
- **Solution**: Ensure `requirements.txt` is in the backend directory
- **Verify**: `ls -la backend/requirements.txt`

### Issue: Port binding error
- **Cause**: Railway might not be setting the PORT environment variable
- **Solution**: Update `start.sh` to use `${PORT:-8000}` as fallback
```bash
uvicorn main:app --host 0.0.0.0 --port ${PORT:-8000}
```

### Issue: Module import errors (e.g., "No module named 'src'")
- **Cause**: Python path or relative imports misconfigured
- **Solution**:
  - Verify `src/` directory exists in backend
  - Ensure `__init__.py` files are in each package directory
  - Check imports in `main.py` use correct relative paths

### Issue: Qdrant or Groq connection fails
- **Cause**: Incorrect API keys or network issues
- **Solution**:
  1. Verify credentials are correct in Railway variables
  2. Test locally first: `python main.py` with local `.env`
  3. Check Qdrant/Groq services are not rate-limiting
  4. View logs: `railway logs --follow`

### Issue: CORS errors from frontend
- **Cause**: `ALLOWED_ORIGINS` doesn't include frontend URL
- **Solution**:
  1. Get your Vercel frontend URL
  2. Update `ALLOWED_ORIGINS` variable: `https://your-frontend.vercel.app`
  3. Redeploy or Railway auto-redeploys on variable change

### Issue: Service keeps restarting
- **Cause**: Application crash or restart policy triggered
- **Solution**:
  1. Check logs: `railway logs --follow`
  2. Look for errors in startup sequence
  3. Verify all required environment variables are set
  4. Test locally before redeploying

---

## Updating Deployment

### When you update the code:

#### Via GitHub (Auto-Deploy)
```bash
git add .
git commit -m "feat: update backend service"
git push origin master
```
Railway automatically detects the push and redeploys.

#### Via Railway CLI
```bash
cd backend
railway up
```

#### Via Dashboard
1. Click your service
2. Click "Deployments"
3. Click "Trigger Deploy" on latest commit

---

## Performance & Monitoring

### Railway Dashboard Features
1. **Metrics**: CPU, memory, network usage
2. **Logs**: Real-time application logs
3. **Deployments**: History and rollback capability
4. **Monitoring**: Alerts and uptime tracking

### Recommended Monitoring
- Set up health check endpoint monitoring
- Monitor error logs for Groq/Qdrant API failures
- Track response latencies (target p95 ≤ 3s)
- Alert on high error rates

---

## Cost Estimation

Railway pricing is based on usage:
- **Free tier**: Limited usage (trial)
- **Pay-as-you-go**: $0.10/GB RAM/hour, $0.10/GB storage/hour
- **Estimate**: Small FastAPI service ~$5-20/month depending on traffic

For cost details, see [railway.app/pricing](https://railway.app/pricing)

---

## Support

- **Railway Docs**: https://docs.railway.app
- **Railway Community**: https://railway.app/community
- **Groq API Docs**: https://console.groq.com/docs/api-overview
- **Qdrant Docs**: https://qdrant.tech/documentation/

