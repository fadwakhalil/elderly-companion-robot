# How to Select Jetson Orin Nano 8GB Variant

**Step-by-step guide to find and select the correct 8GB model in SDK Manager.**

---

## Current Screen: Target Hardware Selection

You're currently on the **Target Hardware** selection screen in Step 01. You can see:
- ✅ "Jetson Orin Nano modules" is selected (has green checkmark)
- ⚠️ Shows "Could not detect a board" (this is normal - you'll connect it later)

**But you need to select the SPECIFIC variant: 8GB vs 4GB**

---

## How to Find the 8GB Option

### Step 1: Click on "Jetson Orin Nano modules"

1. **Click directly on the "Jetson Orin Nano modules" row** (the one with the green checkmark)
2. This should **expand** or show a **dropdown** with specific variants

### Step 2: Look for Sub-Options

After clicking, you should see one of these:

**Option A: Dropdown Menu**
- A dropdown menu appears below "Jetson Orin Nano modules"
- Look for options like:
  - ✅ **"Jetson Orin Nano [8GB developer kit version]"** ← SELECT THIS
  - ❌ "Jetson Orin Nano [4GB developer kit version]" ← NOT THIS

**Option B: Expanded List**
- The list expands to show sub-items
- Look for:
  - ✅ **"Jetson Orin Nano Developer Kit [8GB]"** ← SELECT THIS
  - ❌ "Jetson Orin Nano Developer Kit [4GB]" ← NOT THIS

**Option C: Click the Three Dots (⋯)**
- If you see three dots (⋯) on the right side of "Jetson Orin Nano modules"
- Click those three dots
- This might show a menu with variant options

### Step 3: Select the 8GB Version

1. **Look for any option that says "8GB" or "8 GB"**
2. **Click on it** to select
3. **Avoid any option that says "4GB" or "4 GB"**

---

## What to Look For

### ✅ CORRECT Selection:
- "Jetson Orin Nano [8GB developer kit version]"
- "Jetson Orin Nano Developer Kit [8GB]"
- "Jetson Orin Nano 8GB"
- Any text that includes "8GB" or "8 GB"

### ❌ WRONG Selection:
- "Jetson Orin Nano [4GB developer kit version]"
- "Jetson Orin Nano Developer Kit [4GB]"
- "Jetson Orin Nano 4GB"
- Any text that includes "4GB" or "4 GB"

---

## If You Don't See Sub-Options

**If clicking doesn't show variants, try:**

1. **Double-click** on "Jetson Orin Nano modules"
2. **Right-click** on "Jetson Orin Nano modules" (might show context menu)
3. **Look for an arrow or expand icon** (▶ or ▼) next to the name
4. **Check if there's a dropdown arrow** at the right edge of the selection box

---

## After Selecting 8GB

Once you've selected the 8GB variant:

1. ✅ The green checkmark should move to the 8GB option
2. ✅ The text should clearly show "8GB" somewhere
3. ⚠️ You'll still see "Could not detect a board" - this is **NORMAL**
4. ✅ Continue to the next step (SDK Version selection)

---

## Visual Guide

```
[Target Hardware Selection Screen]

┌─────────────────────────────────────────────┐
│ Target Hardware                             │
├─────────────────────────────────────────────┤
│ ✅ Jetson Orin Nano modules                 │ ← Click here
│    ⚠️ Could not detect a board              │
│    ⋯                                        │
├─────────────────────────────────────────────┤
│   ▼ Jetson Orin Nano modules                │ ← After clicking
│      ✅ Jetson Orin Nano [8GB dev kit...]   │ ← SELECT THIS
│         Jetson Orin Nano [4GB dev kit...]   │ ← NOT THIS
├─────────────────────────────────────────────┤
│   Jetson AGX Thor modules                   │
│   Jetson AGX Orin modules                   │
│   ...                                       │
└─────────────────────────────────────────────┘
```

---

## Still Can't Find It? (No Submenu Appears)

**If clicking on "Jetson Orin Nano modules" shows NO submenu at all:**

**This is normal in some SDK Manager versions!** The variant selection might happen differently.

### ✅ Solution: Continue and Check Flash Configuration

1. **Continue with "Jetson Orin Nano modules" selected**
2. **Go through Step 02 and Step 03** (installation)
3. **When you reach Flash Configuration (Step 04):**
   - Check the **"Recovery mode setup" dropdown**
   - It will show: "Automatic Setup - Jetson Orin Nano 4GB" or "8GB"
   - **This is where you'll see which variant is selected**
   - If it shows "4GB", look for "8GB" option in that dropdown
   - Or check the "Selected Device" dropdown at the top of the screen

**See:** `NO_SUBMENU_SOLUTION.md` for detailed instructions when no submenu appears.

### Alternative: Try These Methods

1. **Click the three-dot icon (⋯)** next to "Jetson Orin Nano modules"
2. **Right-click** on "Jetson Orin Nano modules" (might show context menu)
3. **Look for any text below the selection** that shows "4GB" or "8GB"
4. **Check if there's a dropdown arrow** somewhere on the selection box

---

## ⚠️ Important: Don't Select "Jetson Orin NX"

**You might see "Jetson Orin NX modules" in the list - this is a DIFFERENT model!**

- ❌ **Jetson Orin NX** = Different, more powerful model (not yours)
- ✅ **Jetson Orin Nano** = Your model (what you need to select)

**See:** `JETSON_MODELS_EXPLAINED.md` for details on the difference.

---

## Verification

**After selecting, verify:**
- [ ] The selected option clearly shows "8GB" (not "4GB")
- [ ] Green checkmark is on the 8GB option
- [ ] You can proceed to SDK Version selection

---

**The key is: Look for "8GB" in the text, and make sure it's NOT "4GB"!** 🎯

