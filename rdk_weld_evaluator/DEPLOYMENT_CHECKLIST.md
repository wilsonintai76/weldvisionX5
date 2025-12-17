# Deployment Checklist

**Quick reference checklist for WeldMaster AI Evaluator deployment**

---

## Pre-Deployment

### Hardware
- [ ] RDK X5 board powered on
- [ ] Stereo camera connected (MIPI or USB)
- [ ] Network cable connected (or WiFi configured)
- [ ] Cooling solution installed (heatsink/fan)
- [ ] Power supply adequate (12V 2A minimum)

### Software Prerequisites
- [ ] Ubuntu 20.04/22.04 installed on RDK X5
- [ ] SSH access configured
- [ ] Static IP assigned (recommended)
- [ ] Internet access available (for installation)
- [ ] Git installed
- [ ] Python 3.9+ installed

### Files & Resources
- [ ] Calibration checkerboard pattern (printed)
- [ ] Production configuration files prepared
- [ ] SSH keys generated
- [ ] Backup storage mounted
- [ ] Documentation reviewed

---

## Development Environment Setup (Windows)

### Backend Setup
```powershell
cd "d:\WeldMaster AI Evaluation\rdk_weld_evaluator"
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py --mode test
python main.py --mode server --debug true
```
- [ ] Virtual environment created
- [ ] Dependencies installed
- [ ] Database initialized
- [ ] Server running on port 5000

### Frontend Setup
```powershell
cd "d:\WeldMaster AI Evaluation\weld-frontend"
npm install
npm run dev
```
- [ ] Node modules installed
- [ ] Dev server running on port 3000
- [ ] Browser accessible at http://localhost:3000

### Verification
- [ ] API responds: `curl http://localhost:5000/api/classes`
- [ ] Frontend loads in browser
- [ ] No console errors
- [ ] Mock data displays correctly

---

## Production Deployment (RDK X5)

### System Setup
```bash
ssh ubuntu@<rdk-ip>
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-dev python3-pip python3-venv git
```
- [ ] SSH connection successful
- [ ] Default password changed
- [ ] System updated
- [ ] Required packages installed

### Application Deployment
```bash
cd /opt
sudo git clone https://github.com/wilsonintai76/weldvisionX5.git weld_evaluator
sudo chown -R ubuntu:ubuntu /opt/weld_evaluator
cd /opt/weld_evaluator
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
- [ ] Repository cloned to /opt
- [ ] Permissions set correctly
- [ ] Virtual environment created
- [ ] Python dependencies installed

### Camera Configuration
```bash
ls /dev/video*
python main.py --mode calibrate
```
- [ ] Camera detected
- [ ] Calibration completed (10+ images)
- [ ] Calibration file saved: `config/camera_calibration.yaml`
- [ ] Camera test passed

### Database Setup
```bash
export DATABASE_PATH="/opt/weld_evaluator/production.db"
python main.py --mode test
chmod 660 production.db
```
- [ ] Production database created
- [ ] Tables initialized
- [ ] Permissions set (660)
- [ ] Database accessible

### Backend Service
```bash
sudo nano /etc/systemd/system/weld-evaluator.service
# [Copy service configuration from DEPLOYMENT_INSTRUCTIONS.md]
sudo systemctl daemon-reload
sudo systemctl enable weld-evaluator
sudo systemctl start weld-evaluator
sudo systemctl status weld-evaluator
```
- [ ] Service file created
- [ ] Service enabled
- [ ] Service started
- [ ] Service status: active (running)

### Frontend Build
```bash
cd /opt/weld_evaluator/weld-frontend
npm install
npm run build
ls -lh dist/
```
- [ ] Dependencies installed
- [ ] Production build completed
- [ ] dist/ directory contains files
- [ ] Build size reasonable (<5MB)

### Nginx Setup
```bash
sudo apt install -y nginx
sudo nano /etc/nginx/sites-available/weld-evaluator
# [Copy nginx configuration from DEPLOYMENT_INSTRUCTIONS.md]
sudo ln -s /etc/nginx/sites-available/weld-evaluator /etc/nginx/sites-enabled/
sudo rm /etc/nginx/sites-enabled/default
sudo nginx -t
sudo systemctl reload nginx
sudo systemctl enable nginx
```
- [ ] Nginx installed
- [ ] Configuration file created
- [ ] Symlink created
- [ ] Configuration valid (nginx -t passed)
- [ ] Nginx reloaded

### Firewall Configuration
```bash
sudo ufw allow 22/tcp
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
sudo ufw enable
sudo ufw status
```
- [ ] SSH allowed (22)
- [ ] HTTP allowed (80)
- [ ] HTTPS allowed (443)
- [ ] Firewall enabled

---

## Post-Deployment Verification

### Service Status
```bash
sudo systemctl status weld-evaluator
sudo systemctl status nginx
```
- [ ] weld-evaluator: active (running)
- [ ] nginx: active (running)
- [ ] No error messages in status

### API Tests
```bash
curl http://localhost:5000/api/classes
curl http://localhost:5000/api/students
curl http://localhost:5000/api/health
```
- [ ] /api/classes returns JSON
- [ ] /api/students returns JSON
- [ ] /api/health returns success
- [ ] Response time < 100ms

### Frontend Tests
```bash
curl http://localhost/
curl http://<rdk-ip>/
```
- [ ] Homepage HTML returned
- [ ] No 404 errors
- [ ] Static assets loading
- [ ] Browser access successful

### Integration Tests
From browser at `http://<rdk-ip>/`:
- [ ] Dashboard loads
- [ ] Student Manager displays students
- [ ] Rubric Manager displays rubrics
- [ ] Calibration view accessible
- [ ] Scan view displays camera feed
- [ ] Results view functional

### Database Tests
```bash
sqlite3 production.db ".tables"
sqlite3 production.db "SELECT COUNT(*) FROM student;"
```
- [ ] Tables exist
- [ ] Demo data loaded
- [ ] Queries execute successfully

### Camera Tests
```bash
ls /dev/video*
gst-launch-1.0 v4l2src device=/dev/video0 ! autovideosink
```
- [ ] Camera device present
- [ ] Video stream working
- [ ] Image quality acceptable

### Log Tests
```bash
sudo journalctl -u weld-evaluator -n 20
sudo tail -20 /var/log/nginx/access.log
sudo tail -20 /var/log/nginx/error.log
```
- [ ] Backend logs clean (no errors)
- [ ] Nginx access log shows requests
- [ ] Nginx error log empty or minimal

---

## Security Hardening

### SSH Security
```bash
nano /etc/ssh/sshd_config
# Set: PasswordAuthentication no
sudo systemctl restart sshd
```
- [ ] SSH key authentication configured
- [ ] Password authentication disabled
- [ ] SSH service restarted

### File Permissions
```bash
chmod 600 /opt/weld_evaluator/production.db
chmod 600 /opt/weld_evaluator/config/*.yaml
```
- [ ] Database permissions: 600
- [ ] Config files permissions: 600
- [ ] Ownership: ubuntu:ubuntu

### Service Isolation
- [ ] Service runs as non-root user (ubuntu)
- [ ] Working directory set correctly
- [ ] Environment variables scoped

---

## Backup Setup

### Backup Directory
```bash
mkdir -p /opt/weld_evaluator/backups
mkdir -p /opt/weld_evaluator/scripts
```
- [ ] Backup directory created
- [ ] Scripts directory created

### Backup Script
```bash
nano /opt/weld_evaluator/scripts/backup_database.sh
# [Copy script from DEPLOYMENT_INSTRUCTIONS.md]
chmod +x /opt/weld_evaluator/scripts/backup_database.sh
./scripts/backup_database.sh
```
- [ ] Backup script created
- [ ] Script executable
- [ ] Manual backup successful

### Automated Backups
```bash
crontab -e
# Add: 0 2 * * * /opt/weld_evaluator/scripts/backup_database.sh
```
- [ ] Cron job configured
- [ ] Backup time set (2 AM daily)
- [ ] Crontab saved

### Backup Verification
```bash
ls -lh /opt/weld_evaluator/backups/
```
- [ ] Backup file created
- [ ] File size reasonable
- [ ] Timestamp correct

---

## Monitoring Setup

### Log Rotation
```bash
sudo nano /etc/logrotate.d/weld-evaluator
# [Copy configuration from DEPLOYMENT_INSTRUCTIONS.md]
```
- [ ] Logrotate configuration created
- [ ] Rotation interval: daily
- [ ] Retention: 7 days

### System Monitoring
```bash
sudo apt install -y htop sysstat
sudo systemctl enable sysstat
```
- [ ] htop installed
- [ ] sysstat installed and enabled

---

## Documentation & Handoff

### Documentation Files
- [ ] README.md reviewed
- [ ] DEPLOYMENT_INSTRUCTIONS.md available
- [ ] QUICK_START.md available
- [ ] TECHNICAL_IMPLEMENTATION.md available
- [ ] RDK_X5_DEPLOYMENT_GUIDE.md available

### Contact Information
- [ ] Admin contact recorded
- [ ] Support email configured
- [ ] Escalation path documented

### Training
- [ ] Operations team trained
- [ ] Admin credentials shared securely
- [ ] Common procedures demonstrated

---

## Final Verification

### Functional Tests
1. **Create Student**: Add new student via UI
   - [ ] Student saved to database
   - [ ] Appears in Student Manager

2. **Perform Evaluation**: Run complete scan workflow
   - [ ] Camera feed displays
   - [ ] Weld analysis completes
   - [ ] Score displayed
   - [ ] Results saved

3. **View Statistics**: Check class/student statistics
   - [ ] Statistics calculated correctly
   - [ ] Charts/graphs display

4. **Export Data**: Download CSV export
   - [ ] CSV file generated
   - [ ] Data complete and accurate

### Performance Tests
```bash
time curl http://localhost:5000/api/classes
# Should complete in < 100ms

ab -n 100 -c 10 http://localhost/api/classes
# Check concurrent request handling
```
- [ ] Single request: < 100ms
- [ ] Concurrent requests handled
- [ ] No errors under load

### Recovery Tests
```bash
sudo systemctl stop weld-evaluator
# Wait 10 seconds
sudo systemctl start weld-evaluator
# Verify service recovers
```
- [ ] Service stops cleanly
- [ ] Service starts successfully
- [ ] Data persisted correctly

---

## Sign-Off

### Deployment Team
- [ ] Deployment completed by: _________________ Date: _________
- [ ] Verification completed by: _________________ Date: _________
- [ ] Approved by: _________________ Date: _________

### Notes
```
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________
```

### Issues Encountered
```
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________
```

### Next Steps
- [ ] Schedule follow-up check (1 week)
- [ ] Monitor logs for anomalies
- [ ] Gather user feedback
- [ ] Plan future enhancements

---

**Deployment Status:** 
- [ ] Development Environment Ready
- [ ] Staging Environment Ready
- [ ] Production Environment Ready
- [ ] All Tests Passed
- [ ] Documentation Complete
- [ ] **PRODUCTION DEPLOYMENT APPROVED**

---

*Last Updated: December 17, 2025*  
*Document Version: 1.0*
