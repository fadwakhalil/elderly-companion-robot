# How to Reformat WD My Passport on Mac

**Step-by-step guide to reformat your WD My Passport USB drive for creating a bootable Ubuntu USB.**

---

## ⚠️ Important Warning

**Reformatting will ERASE ALL DATA on the drive!**
- Backup any important files before proceeding
- This process cannot be undone
- Make sure you've selected the correct drive

---

## Method 1: Using Disk Utility (GUI - Easier)

### Step 1: Open Disk Utility

1. **Open Finder**
2. **Go to Applications → Utilities → Disk Utility**
   - Or press `Cmd+Space` and type "Disk Utility"

### Step 2: Select Your WD My Passport

1. **In the left sidebar**, look for "WD My Passport" or similar
2. **Click on the drive name** (the top-level entry, not any partitions)
   - It should show the full capacity (e.g., "WD My Passport 1TB")
   - Make sure it's the external drive, not your Mac's internal drive!

### Step 3: Erase the Drive

1. **Click the "Erase" button** at the top of the window

2. **Fill in the erase options:**
   - **Name:** `USBDRIVE` (or any name you want)
   - **Format:** Select **"MS-DOS (FAT32)"** or **"ExFAT"**
     - **FAT32** is recommended for bootable USB drives
     - **ExFAT** works too, but FAT32 is more compatible
   - **Scheme:** Select **"GUID Partition Map"** (for Intel Macs) or **"Apple Partition Map"** (for older Macs)
     - For modern Macs, use **"GUID Partition Map"**

3. **Click "Erase"** button

4. **Confirm the warning:**
   - A popup will warn you that all data will be erased
   - Click **"Erase"** to confirm

5. **Wait for completion:**
   - This may take a few minutes depending on drive size
   - You'll see a progress bar

6. **When done:**
   - You'll see "Erase complete"
   - The drive will be reformatted and ready to use

### Step 4: Verify

1. **Check the drive appears in Finder**
2. **It should be empty and ready for use**

---

## Method 2: Using Terminal (Command Line - Faster)

### Step 1: Find Your WD My Passport

```bash
# List all disks
diskutil list
```

**Look for your WD My Passport in the output:**
- It will show as `/dev/diskX` where X is a number (e.g., `/dev/disk2`)
- Look for "external, physical" and the size matching your drive
- **Example output:**
  ```
  /dev/disk2 (external, physical):
     #:                       TYPE NAME                    SIZE       IDENTIFIER
     0:     FDisk_partition_scheme                        *1.0 TB     disk2
     1:                 DOS_FAT_32 WD My Passport          1.0 TB     disk2s1
  ```

**⚠️ IMPORTANT:** Note the disk number (e.g., `disk2`) - you'll need this!

### Step 2: Unmount the Drive

```bash
# Replace X with your disk number (e.g., disk2)
sudo diskutil unmountDisk /dev/diskX
```

**Example:**
```bash
sudo diskutil unmountDisk /dev/disk2
```

### Step 3: Erase and Format

**Option A: Format as FAT32 (Recommended for bootable USB):**

```bash
# Replace X with your disk number
sudo diskutil eraseDisk FAT32 USBDRIVE /dev/diskX
```

**Example:**
```bash
sudo diskutil eraseDisk FAT32 USBDRIVE /dev/disk2
```

**Option B: Format as ExFAT (Alternative):**

```bash
# Replace X with your disk number
sudo diskutil eraseDisk ExFAT USBDRIVE /dev/diskX
```

### Step 4: Wait for Completion

- The command will show progress
- When done, you'll see: "Finished erase on diskX"
- The drive is now reformatted and ready

### Step 5: Verify

```bash
# Check the drive is formatted correctly
diskutil list
```

**You should see:**
- The drive with the new name (e.g., "USBDRIVE")
- Format as "DOS_FAT_32" or "ExFAT"

---

## Quick Reference Commands

**Complete process in one go:**

```bash
# 1. List disks to find your WD My Passport
diskutil list

# 2. Replace diskX with your actual disk number (e.g., disk2)
#    This will unmount, erase, and format in one command
sudo diskutil eraseDisk FAT32 USBDRIVE /dev/diskX

# 3. Verify it worked
diskutil list
```

---

## Troubleshooting

### "Drive is too large" Error (FAT32 Limit)

**If you get: "Drive is too large, the number of blocks is larger than any FAT FS can support"**

**Problem:** FAT32 has a maximum partition size of 32 GB. Your 3 TB drive is too large.

**Solution: Create a smaller partition instead of formatting the entire drive:**

1. **In Disk Utility:**
   - Select your WD My Passport drive
   - Click **"Partition"** (instead of "Erase")
   - Click **"Partition Layout"** → Select **"1 Partition"**
   - Click the partition in the pie chart
   - Set **Size:** `32 GB` (or `16 GB` - enough for Ubuntu ISO)
   - Set **Format:** `MS-DOS (FAT32)`
   - Set **Name:** `USBDRIVE`
   - Click **"Apply"**
   - This creates a 32 GB FAT32 partition, leaving the rest unallocated

2. **Or use Terminal to create a smaller partition:**
   ```bash
   # Find your drive
   diskutil list
   
   # Create a 32 GB FAT32 partition (replace diskX with your disk number)
   sudo diskutil partitionDisk /dev/diskX 1 MBR FAT32 USBDRIVE 32G
   ```

**Alternative: Don't format at all!**
- You can skip formatting entirely
- balenaEtcher or `dd` will write the ISO directly to the drive
- The ISO includes its own partition table and file system
- Just erase the drive (without formatting) and proceed with bootable USB creation

### "Resource busy" or "Disk is in use" Error

**Solution:**
```bash
# Force unmount first
sudo diskutil unmountDisk force /dev/diskX

# Then try erase again
sudo diskutil eraseDisk FAT32 USBDRIVE /dev/diskX
```

### Can't Find the Drive

1. **Make sure the drive is connected:**
   - Check USB cable
   - Try a different USB port
   - Try a different cable

2. **Check if it appears in Disk Utility:**
   - Open Disk Utility
   - Look in the left sidebar
   - If it doesn't appear, the drive may have a hardware issue

### Wrong Disk Selected

**⚠️ CRITICAL:** Always double-check the disk number!

- **Check the size** matches your WD My Passport
- **Check it says "external, physical"**
- **Never use your Mac's internal drive** (usually `/dev/disk0` or `/dev/disk1`)

### Format Fails

1. **Try using Disk Utility GUI instead**
2. **Check disk for errors:**
   ```bash
   diskutil verifyDisk /dev/diskX
   ```
3. **If errors found, repair first:**
   ```bash
   diskutil repairDisk /dev/diskX
   ```

---

## After Reformatting

**Your WD My Passport is now ready for:**
- Creating a bootable Ubuntu USB (using balenaEtcher or `dd`)
- General file storage
- Use with any operating system (FAT32/ExFAT are cross-platform)

**Next Steps:**
- Go back to [BOOTABLE_USB_FLASH_METHOD.md](BOOTABLE_USB_FLASH_METHOD.md)
- Try creating the bootable USB again with balenaEtcher
- Or use the command line `dd` method

---

## Format Options Explained

| Format | Best For | Max File Size | Compatibility |
|--------|----------|---------------|---------------|
| **FAT32** | Bootable USB drives | 4 GB per file | Windows, Mac, Linux |
| **ExFAT** | Large files, cross-platform | No limit | Windows, Mac, Linux |
| **APFS** | Mac-only | No limit | Mac only |
| **NTFS** | Windows | No limit | Windows (read-only on Mac) |

**For bootable Ubuntu USB: Use FAT32 or ExFAT**

---

**Ready to reformat? Follow Method 1 (GUI) for easiest process, or Method 2 (Terminal) for faster command-line approach!**

