# 🔧 QUICK FIX: Translation Toggle Not Visible

## The Problem
Your browser is showing OLD cached JavaScript. The component IS in the code (I can see it), but your browser hasn't loaded the new version.

## ✅ Solution (3 Steps)

### Step 1: Stop Frontend Server
In the terminal where your frontend server is running:
- Press **`Ctrl + C`** to stop it

### Step 2: Restart Frontend Server

**Option A: Use the restart script (easiest)**
```bash
./restart-frontend.sh
```

**Option B: Manual restart**
```bash
cd frontend
rm -rf .docusaurus build
npm start
```

### Step 3: Hard Refresh Browser
- **Windows/Linux**: `Ctrl + Shift + R`
- **Mac**: `Cmd + Shift + R`

**OR** in DevTools:
1. Press `F12` to open DevTools
2. Right-click the refresh button
3. Click **"Empty Cache and Hard Reload"**

## What to Expect

After restarting and refreshing:
- ✅ You should see **yellow debug box** in top-right corner
- ✅ Console should show: `"TranslationToggle render:"`
- ✅ Checkbox labeled "Translate to Urdu" should be visible

## If Still Not Working

1. **Check Console** (F12 → Console tab):
   - Look for red errors
   - Look for `"TranslationToggle render:"` log

2. **Check Network Tab** (F12 → Network tab):
   - Refresh page
   - Look for `chapter.tsx` or `TranslationToggle.tsx` in the list
   - Check if they're loading successfully (status 200)

3. **Try Incognito Window**:
   - Open a private/incognito window
   - Navigate to: `http://localhost:3000/physical-ai-humanoid-robotics-book/chapter?chapter_id=intro`
   - This bypasses all cache

## Most Common Fix

**99% of the time, this works:**
```bash
# Stop server (Ctrl+C)
cd frontend
npm start
# Wait for compilation, then Ctrl+Shift+R in browser
```

That's it! 🎉

