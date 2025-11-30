# Jetson Models Explained: Orin Nano vs Orin NX

**Quick guide to understand the difference and which one you need.**

---

## ⚠️ Important: You Have Jetson Orin Nano 8GB

**Based on your hardware setup, you have:**
- ✅ **Jetson Orin Nano 8GB Developer Kit** ← This is what you need to select

**You should NOT select:**
- ❌ Jetson Orin NX (different model)
- ❌ Jetson Orin Nano 4GB (wrong memory size)

---

## Jetson Orin Nano vs Jetson Orin NX

### Jetson Orin Nano (What You Have) ✅

**Your Model:**
- **Name:** Jetson Orin Nano Developer Kit
- **Memory:** 8GB LPDDR5
- **AI Performance:** Up to 40 TOPS
- **Power:** 7W-15W
- **Form Factor:** Developer Kit (larger, with ports and connectors)
- **Price:** Lower cost, entry-level
- **Use Case:** Perfect for your companion robot project

**What it looks like:**
- Larger board with visible ports (USB, Ethernet, HDMI, etc.)
- Designed for development and prototyping
- Easy to connect peripherals

---

### Jetson Orin NX (Different Model) ❌

**Different Model:**
- **Name:** Jetson Orin NX Module
- **Memory:** 8GB or 16GB LPDDR5
- **AI Performance:** Up to 70-100 TOPS (more powerful)
- **Power:** 10W-25W
- **Form Factor:** Module (smaller, needs carrier board)
- **Price:** Higher cost, more powerful
- **Use Case:** More advanced applications requiring higher performance

**What it looks like:**
- Smaller module (needs a carrier board to use)
- More compact, designed for production
- Requires additional hardware to connect peripherals

---

## Key Differences Summary

| Feature | Jetson Orin Nano (Yours) | Jetson Orin NX |
|---------|-------------------------|----------------|
| **Model Type** | Developer Kit | Module |
| **Memory Options** | 4GB or 8GB | 8GB or 16GB |
| **AI Performance** | ~40 TOPS | ~70-100 TOPS |
| **Power Consumption** | 7W-15W | 10W-25W |
| **Form Factor** | Larger, standalone | Smaller, needs carrier |
| **Price** | Lower | Higher |
| **Your Hardware** | ✅ **YES - This is yours!** | ❌ No |

---

## In SDK Manager: What to Select

### ✅ CORRECT Selection:
1. **Click on:** "Jetson Orin Nano modules"
2. **Then select:** "Jetson Orin Nano [8GB developer kit version]"
   - OR "Jetson Orin Nano Developer Kit [8GB]"
   - Must say "8GB" and "Nano" (not "NX")

### ❌ WRONG Selections:
- ❌ "Jetson Orin NX modules" - Different model!
- ❌ "Jetson Orin Nano [4GB developer kit version]" - Wrong memory size!
- ❌ Any option with "NX" in the name - Different model!

---

## Why This Matters

**Selecting the wrong model will cause:**
- ❌ Flashing will fail or produce errors
- ❌ Wrong drivers and software will be installed
- ❌ Your Jetson won't work properly
- ❌ You'll need to redo the entire flashing process

**Selecting the correct model ensures:**
- ✅ Correct JetPack version for your hardware
- ✅ Proper drivers and software
- ✅ Your Jetson will work as expected
- ✅ Smooth setup process

---

## Quick Checklist

When selecting in SDK Manager:

- [ ] I see "Jetson Orin Nano modules" in the list
- [ ] I click on it to expand/show variants
- [ ] I select the option that says "8GB" (not "4GB")
- [ ] I select the option that says "Nano" (not "NX")
- [ ] The selected option says "developer kit" or "dev kit"
- [ ] I do NOT select "Jetson Orin NX" (different model)

---

## Visual Guide

```
[SDK Manager Target Hardware List]

┌─────────────────────────────────────────────┐
│ Target Hardware                             │
├─────────────────────────────────────────────┤
│ ✅ Jetson Orin Nano modules                 │ ← Click this
│    ▼ Jetson Orin Nano [8GB dev kit...]     │ ← Select this ✅
│       Jetson Orin Nano [4GB dev kit...]    │ ← NOT this ❌
├─────────────────────────────────────────────┤
│   Jetson Orin NX modules                    │ ← NOT this ❌
│   (Different model - more powerful)         │
├─────────────────────────────────────────────┤
│   Jetson AGX Orin modules                   │
│   Jetson AGX Xavier modules                 │
│   ...                                       │
└─────────────────────────────────────────────┘
```

---

## Summary

**You have:** Jetson Orin Nano 8GB Developer Kit

**In SDK Manager, select:**
- ✅ "Jetson Orin Nano modules" → "Jetson Orin Nano [8GB developer kit version]"

**Do NOT select:**
- ❌ "Jetson Orin NX" (different, more powerful model)
- ❌ "Jetson Orin Nano [4GB]" (wrong memory size)

**The key is: "Nano" + "8GB" = Your hardware!** 🎯

