# Deployment Troubleshooting Guide

**Quick solutions for common deployment issues**

---

## Table of Contents

1. [Backend Issues](#backend-issues)
2. [Frontend Issues](#frontend-issues)
3. [Database Issues](#database-issues)
4. [Camera Issues](#camera-issues)
5. [Network Issues](#network-issues)
6. [Performance Issues](#performance-issues)
7. [Service Management](#service-management)
8. [Emergency Procedures](#emergency-procedures)

---

## Backend Issues

### Issue: Backend service won't start

**Symptoms:**
```bash
sudo systemctl status weld-evaluator
# Output: Failed to start weld-evaluator service
```

**Solutions:**

1. **Check Python environment**
   ```bash
   /opt/weld_evaluator/venv/bin/python --version
   # Should show Python 3.9+
   ```

2. **Verify dependencies installed**
   ```bash
   source /opt/weld_evaluator/venv/bin/activate
   pip list | grep -E "flask|sqlalchemy|opencv"
   ```
   If missing:
   ```bash
   pip install -r /opt/weld_evaluator/requirements.txt
   ```

3. **Check service file syntax**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl start weld-evaluator
   sudo journalctl -u weld-evaluator -n 50
   ```

4. **Check permissions**
   ```bash
   ls -l /opt/weld_evaluator/main.py
   # Should be readable by ubuntu user
   sudo chown -R ubuntu:ubuntu /opt/weld_evaluator
   ```

### Issue: Port 5000 already in use

**Symptoms:**
```
Error: Address already in use
```

**Solutions:**

1. **Find process using port**
   ```bash
   sudo lsof -i :5000
   # Or
   sudo netstat -tulpn | grep :5000
   ```

2. **Kill conflicting process**
   ```bash
   sudo kill -9 <PID>
   ```

3. **Change port in configuration**
   ```bash
   # Edit main.py or config file
   python main.py --mode server --port 5001
   ```

### Issue: Module import errors

**Symptoms:**
```
ModuleNotFoundError: No module named 'flask'
```

**Solutions:**

1. **Activate virtual environment**
   ```bash
   source /opt/weld_evaluator/venv/bin/activate
   which python
   # Should show: /opt/weld_evaluator/venv/bin/python
   ```

2. **Reinstall dependencies**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt --force-reinstall
   ```

3. **Verify service file uses correct Python**
   ```bash
   sudo nano /etc/systemd/system/weld-evaluator.service
   # ExecStart should use: /opt/weld_evaluator/venv/bin/python
   ```

### Issue: API returns 500 errors

**Symptoms:**
```bash
curl http://localhost:5000/api/classes
# {"error": "Internal Server Error"}
```

**Solutions:**

1. **Check application logs**
   ```bash
   sudo journalctl -u weld-evaluator -f
   # Watch for Python tracebacks
   ```

2. **Enable debug mode temporarily**
   ```bash
   sudo systemctl stop weld-evaluator
   source /opt/weld_evaluator/venv/bin/activate
   python main.py --mode server --debug true
   # Check console output for errors
   ```

3. **Verify database accessible**
   ```bash
   ls -l /opt/weld_evaluator/production.db
   sqlite3 production.db ".tables"
   ```

---

## Frontend Issues

### Issue: Blank page after deployment

**Symptoms:**
- Browser shows blank white page
- No console errors
- Network tab shows 200 OK responses

**Solutions:**

1. **Check nginx configuration**
   ```bash
   sudo nginx -t
   # Should show "test is successful"
   ```

2. **Verify dist files exist**
   ```bash
   ls -lh /opt/weld_evaluator/weld-frontend/dist/
   # Should show index.html and assets/
   ```

3. **Check nginx root path**
   ```bash
   sudo cat /etc/nginx/sites-enabled/weld-evaluator | grep root
   # Should match dist directory location
   ```

4. **Rebuild frontend**
   ```bash
   cd /opt/weld_evaluator/weld-frontend
   rm -rf dist/
   npm run build
   ls -lh dist/
   sudo systemctl reload nginx
   ```

### Issue: API calls return 404

**Symptoms:**
```
Failed to load resource: http://<ip>/api/classes 404
```

**Solutions:**

1. **Check nginx proxy configuration**
   ```bash
   sudo cat /etc/nginx/sites-enabled/weld-evaluator | grep -A 5 "location /api"
   ```

2. **Verify backend is running**
   ```bash
   curl http://localhost:5000/api/classes
   # Should return JSON
   ```

3. **Test nginx proxy**
   ```bash
   curl http://localhost/api/classes
   # Should also return JSON
   ```

4. **Check nginx error logs**
   ```bash
   sudo tail -50 /var/log/nginx/error.log
   ```

### Issue: Static assets not loading (CSS/JS)

**Symptoms:**
- Page loads but styling broken
- Console shows "Failed to load resource: 404"

**Solutions:**

1. **Check asset paths in build**
   ```bash
   cat /opt/weld_evaluator/weld-frontend/dist/index.html | grep assets
   # Paths should be relative: /assets/...
   ```

2. **Verify nginx serves assets**
   ```bash
   curl -I http://localhost/assets/index-abc123.js
   # Should return 200 OK
   ```

3. **Clear browser cache**
   - Press Ctrl+Shift+R (hard refresh)
   - Or clear cache in DevTools

4. **Rebuild with correct base path**
   ```bash
   cd /opt/weld_evaluator/weld-frontend
   # Check vite.config.js base setting
   npm run build
   ```

---

## Database Issues

### Issue: Database locked

**Symptoms:**
```
sqlite3.OperationalError: database is locked
```

**Solutions:**

1. **Check for multiple connections**
   ```bash
   lsof /opt/weld_evaluator/production.db
   ```

2. **Restart backend service**
   ```bash
   sudo systemctl restart weld-evaluator
   ```

3. **Check database integrity**
   ```bash
   sqlite3 production.db "PRAGMA integrity_check;"
   # Should return "ok"
   ```

4. **Last resort: backup and rebuild**
   ```bash
   cp production.db production.db.backup
   sqlite3 production.db.backup ".dump" | sqlite3 production.db.new
   mv production.db production.db.old
   mv production.db.new production.db
   ```

### Issue: Database not found

**Symptoms:**
```
sqlite3.OperationalError: unable to open database file
```

**Solutions:**

1. **Check file exists**
   ```bash
   ls -l /opt/weld_evaluator/production.db
   ```

2. **Check permissions**
   ```bash
   chmod 660 /opt/weld_evaluator/production.db
   chown ubuntu:ubuntu /opt/weld_evaluator/production.db
   ```

3. **Check directory permissions**
   ```bash
   ls -ld /opt/weld_evaluator
   # Should be writable by ubuntu user
   ```

4. **Recreate database**
   ```bash
   cd /opt/weld_evaluator
   source venv/bin/activate
   python main.py --mode test
   ```

### Issue: Database schema mismatch

**Symptoms:**
```
sqlite3.OperationalError: no such column: ...
```

**Solutions:**

1. **Check database schema**
   ```bash
   sqlite3 production.db ".schema"
   ```

2. **Compare with expected schema**
   ```bash
   python -c "from src.database_sqlalchemy import db; db.create_all()"
   ```

3. **Backup and migrate**
   ```bash
   # Export data
   sqlite3 production.db ".mode csv" ".output data.csv" "SELECT * FROM evaluation;"
   
   # Recreate database
   mv production.db production.db.old
   python main.py --mode test
   
   # Import data (if schema compatible)
   sqlite3 production.db ".mode csv" ".import data.csv evaluation"
   ```

---

## Camera Issues

### Issue: Camera not detected

**Symptoms:**
```bash
ls /dev/video*
# No devices found
```

**Solutions:**

1. **Check USB connection**
   ```bash
   lsusb
   dmesg | tail -20
   # Look for USB device messages
   ```

2. **Check MIPI connection (RDK X5)**
   ```bash
   cat /sys/class/video4linux/*/name
   # Or follow RDK documentation for MIPI cameras
   ```

3. **Load camera driver**
   ```bash
   sudo modprobe v4l2loopback
   # Or RDK-specific driver
   ```

4. **Reboot system**
   ```bash
   sudo reboot
   ```

### Issue: Poor image quality

**Symptoms:**
- Images dark/overexposed
- Blurry images
- Color incorrect

**Solutions:**

1. **Adjust camera parameters**
   ```bash
   # Edit config/vision_config.yaml
   camera:
     exposure: auto  # Try manual if auto fails
     gain: 1.0       # Adjust 1.0-4.0
     white_balance: auto
   ```

2. **Test camera directly**
   ```bash
   v4l2-ctl --device=/dev/video0 --list-ctrls
   v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_auto=1
   v4l2-ctl --device=/dev/video0 --set-ctrl=gain=100
   ```

3. **Check physical camera**
   - Clean lens
   - Adjust focus
   - Verify lighting conditions

### Issue: Calibration fails

**Symptoms:**
```
Error: Checkerboard pattern not found
```

**Solutions:**

1. **Verify checkerboard specifications**
   - Use correct size (e.g., 9x6 inner corners)
   - Print with good contrast
   - Mount on flat, rigid surface

2. **Improve lighting**
   - Add diffuse lighting
   - Avoid shadows on pattern
   - Prevent glare/reflections

3. **Adjust capture settings**
   ```bash
   # In calibration code, try:
   - Larger checkerboard
   - More contrast
   - Different distances/angles
   ```

4. **Use more images**
   - Capture 15-20 images minimum
   - Vary positions and angles
   - Cover full field of view

---

## Network Issues

### Issue: Cannot access from external network

**Symptoms:**
- Can access from RDK X5 (localhost)
- Cannot access from other computers

**Solutions:**

1. **Check firewall**
   ```bash
   sudo ufw status
   # Ensure port 80 is allowed
   sudo ufw allow 80/tcp
   sudo ufw reload
   ```

2. **Verify nginx binding**
   ```bash
   sudo netstat -tulpn | grep :80
   # Should show 0.0.0.0:80 (not 127.0.0.1:80)
   ```

3. **Check nginx server_name**
   ```bash
   sudo cat /etc/nginx/sites-enabled/weld-evaluator | grep server_name
   # Should be: server_name _;
   ```

4. **Test from RDK X5**
   ```bash
   curl http://<rdk-ip>/
   # If this works but external doesn't, check router/firewall
   ```

### Issue: CORS errors in browser

**Symptoms:**
```
Access to XMLHttpRequest blocked by CORS policy
```

**Solutions:**

1. **Enable CORS in Flask**
   ```python
   # src/api/app.py
   from flask_cors import CORS
   CORS(app, resources={r"/api/*": {"origins": "*"}})
   ```

2. **Configure nginx CORS headers**
   ```nginx
   location /api {
       add_header 'Access-Control-Allow-Origin' '*';
       add_header 'Access-Control-Allow-Methods' 'GET, POST, PUT, DELETE, OPTIONS';
       proxy_pass http://127.0.0.1:5000;
   }
   ```

3. **Restart services**
   ```bash
   sudo systemctl restart weld-evaluator
   sudo systemctl reload nginx
   ```

---

## Performance Issues

### Issue: Slow API responses

**Symptoms:**
- Requests take >1 second
- Frontend feels sluggish

**Solutions:**

1. **Check CPU usage**
   ```bash
   top -bn1 | head -20
   # Look for high CPU processes
   ```

2. **Check memory**
   ```bash
   free -h
   # Ensure sufficient free memory
   ```

3. **Profile backend**
   ```bash
   python -m cProfile -s cumulative main.py --mode demo > profile.txt
   cat profile.txt | head -30
   ```

4. **Optimize database queries**
   ```python
   # Add indexes to frequently queried columns
   # Use eager loading for relationships
   # Cache repeated queries
   ```

5. **Reduce image resolution**
   ```yaml
   # config/vision_config.yaml
   stereo:
     image_width: 1280  # Down from 1920
     image_height: 720  # Down from 1080
   ```

### Issue: High memory usage

**Symptoms:**
```bash
free -h
# Shows very low available memory
```

**Solutions:**

1. **Check process memory**
   ```bash
   ps aux --sort=-%mem | head -10
   ```

2. **Restart services**
   ```bash
   sudo systemctl restart weld-evaluator
   ```

3. **Add swap space (if needed)**
   ```bash
   sudo fallocate -l 2G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   # Make permanent in /etc/fstab
   ```

4. **Optimize code**
   - Release large arrays after use
   - Use generators instead of lists
   - Limit concurrent requests

---

## Service Management

### Issue: Service fails to start on boot

**Symptoms:**
- Manual start works
- Auto-start after reboot fails

**Solutions:**

1. **Check service enabled**
   ```bash
   sudo systemctl is-enabled weld-evaluator
   # Should show "enabled"
   sudo systemctl enable weld-evaluator
   ```

2. **Check service dependencies**
   ```bash
   sudo nano /etc/systemd/system/weld-evaluator.service
   # Verify:
   [Unit]
   After=network.target
   ```

3. **Test after reboot**
   ```bash
   sudo reboot
   # Wait for boot
   ssh ubuntu@<rdk-ip>
   sudo systemctl status weld-evaluator
   ```

### Issue: Service restarts continuously

**Symptoms:**
```bash
sudo systemctl status weld-evaluator
# Shows many restart attempts
```

**Solutions:**

1. **Check logs for error**
   ```bash
   sudo journalctl -u weld-evaluator -n 100
   ```

2. **Adjust restart policy**
   ```bash
   sudo nano /etc/systemd/system/weld-evaluator.service
   [Service]
   Restart=on-failure  # Instead of 'always'
   RestartSec=30       # Longer delay
   ```

3. **Fix underlying issue**
   - Check dependencies
   - Verify configuration
   - Fix code errors

---

## Emergency Procedures

### Complete System Reset

**When to use:** System completely broken, need fresh start

```bash
# 1. Backup data
sudo systemctl stop weld-evaluator
cp /opt/weld_evaluator/production.db ~/production.db.backup
tar -czf ~/config_backup.tar.gz /opt/weld_evaluator/config

# 2. Remove application
sudo systemctl stop weld-evaluator
sudo systemctl disable weld-evaluator
sudo rm /etc/systemd/system/weld-evaluator.service
sudo rm -rf /opt/weld_evaluator

# 3. Redeploy from scratch
# Follow DEPLOYMENT_INSTRUCTIONS.md

# 4. Restore data
cp ~/production.db.backup /opt/weld_evaluator/production.db
tar -xzf ~/config_backup.tar.gz -C /
```

### Rollback to Previous Version

**When to use:** New deployment causing issues

```bash
# 1. Stop services
sudo systemctl stop weld-evaluator
sudo systemctl stop nginx

# 2. Checkout previous version
cd /opt/weld_evaluator
git log --oneline -10
git checkout <previous-commit>

# 3. Reinstall dependencies
source venv/bin/activate
pip install -r requirements.txt

# 4. Rebuild frontend
cd weld-frontend
npm install
npm run build

# 5. Restart services
sudo systemctl start weld-evaluator
sudo systemctl start nginx

# 6. Verify
curl http://localhost/api/classes
```

### Emergency Database Recovery

**When to use:** Database corrupted or lost

```bash
# 1. Try to recover from backup
ls -lh /opt/weld_evaluator/backups/
cp backups/production_YYYYMMDD_HHMMSS.db production.db

# 2. If no backup, rebuild
mv production.db production.db.corrupted
python main.py --mode test

# 3. Attempt data recovery from corrupted DB
sqlite3 production.db.corrupted ".dump" > dump.sql
sqlite3 production.db < dump.sql
```

---

## Getting Additional Help

### Before Opening Issue

1. **Collect system information**
   ```bash
   uname -a > system_info.txt
   sudo systemctl status weld-evaluator >> system_info.txt
   sudo journalctl -u weld-evaluator -n 100 >> system_info.txt
   ```

2. **Collect logs**
   ```bash
   sudo journalctl -u weld-evaluator --since "1 hour ago" > app_logs.txt
   sudo cat /var/log/nginx/error.log > nginx_error.log
   ```

3. **Document steps to reproduce**
   - What were you trying to do?
   - What steps did you follow?
   - What was the expected result?
   - What actually happened?

### Support Resources

- **Documentation**: Check all .md files in repository
- **GitHub Issues**: Search existing issues first
- **Logs**: Always include relevant log excerpts
- **System Info**: Include OS version, Python version, hardware specs

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025

*For deployment procedures, see [DEPLOYMENT_INSTRUCTIONS.md](DEPLOYMENT_INSTRUCTIONS.md)*  
*For technical details, see [TECHNICAL_IMPLEMENTATION.md](TECHNICAL_IMPLEMENTATION.md)*
