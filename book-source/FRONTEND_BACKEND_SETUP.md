# Frontend-Backend Integration Setup

## What Was Updated

✅ **ChatBot.tsx** - Updated to use environment variables for backend URL
✅ **API Payload** - Changed to match backend format (query + chat_history)
✅ **Response Handling** - Maps backend response to frontend format

---

## Setup for Vercel Deployment

### Step 1: Update Vercel Environment Variables

1. Go to your **Vercel Project**
2. Click **Settings** → **Environment Variables**
3. Add this variable:

```
REACT_APP_API_BASE_URL=https://shajar5110-hackathon-ai-book.hf.space
```

4. Click **Save**
5. Redeploy your project

### Step 2: Redeploy Frontend on Vercel

```bash
# In your book-source directory
npm run build
git add .
git commit -m "Connect frontend to HF Spaces backend"
git push
```

Or just push to GitHub and let Vercel auto-deploy.

---

## Setup for Local Development

If testing locally, create `.env.local` in `book-source/` folder:

```env
REACT_APP_API_BASE_URL=http://localhost:8000
```

Then run:
```bash
cd book-source
npm start
```

---

## Backend URL

**Production**: `https://shajar5110-hackathon-ai-book.hf.space`
**Local Dev**: `http://localhost:8000`

---

## API Integration

### Request Format (Frontend → Backend)

```javascript
{
  "query": "Your question here",
  "chat_history": []
}
```

### Response Format (Backend → Frontend)

```json
{
  "response": "Answer from LLM",
  "sources": [
    {
      "chapter": "Chapter 1: Introduction to Physical AI",
      "module": "Module 1.1: Fundamentals",
      "section": "what-is-physical-ai",
      "excerpt": "Physical AI is...",
      "similarity": 0.95
    }
  ],
  "confidence": "high",
  "processing_time_ms": 1250
}
```

---

## Troubleshooting

### CORS Error
- ✅ Already configured on backend
- Backend allows: `https://physical-ai-humanoid-robotics-book-psi.vercel.app`
- If using different URL, update `ALLOWED_ORIGINS` in backend environment variables

### Chat Returns Empty
- Check backend logs: https://huggingface.co/spaces/shajar5110/hackathon-Ai-book → Logs
- Verify Qdrant and Groq API keys are set

### No Response from Backend
- Verify `REACT_APP_API_BASE_URL` is set correctly
- Test with: `curl https://shajar5110-hackathon-ai-book.hf.space/health`

### API Docs
- View: `https://shajar5110-hackathon-ai-book.hf.space/api/docs`

---

## Files Modified

- `book-source/src/components/ChatBot.tsx` - Updated API integration
- Vercel environment variables (set manually in Vercel UI)

---

## Testing Steps

1. **Local Testing**
   ```bash
   cd book-source
   REACT_APP_API_BASE_URL=http://localhost:8000 npm start
   ```
   Then ask a question in the chatbot widget

2. **Production Testing (After Vercel Deploy)**
   - Open your Vercel URL
   - Click the 💬 button
   - Ask a question
   - Should get response from HF Spaces backend

3. **Check Network Tab (Browser DevTools)**
   - F12 → Network tab
   - Send a message
   - Look for POST request to `/api/chat`
   - Check Response tab for the answer

---

## Backend API Endpoints

- **Health**: `GET /health`
- **Chat**: `POST /api/chat`
- **Docs**: `GET /api/docs`
- **ReDoc**: `GET /api/redoc`

---

**You're All Set!** 🚀

After setting the environment variable in Vercel and redeploying, your frontend will be connected to your backend!
