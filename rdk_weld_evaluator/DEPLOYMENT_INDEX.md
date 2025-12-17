# Deployment Documentation Index

**Quick reference to all deployment resources for WeldMaster AI Evaluator**

---

## Core Deployment Documents

### 1. [DEPLOYMENT_INSTRUCTIONS.md](DEPLOYMENT_INSTRUCTIONS.md) (29 KB)
**Primary deployment guide with complete step-by-step procedures**

**Contents:**
- Prerequisites (hardware & software requirements)
- Environment overview (development, staging, production)
- Development deployment (Windows setup)
- Production deployment (RDK X5 setup)
- Database deployment & migrations
- Frontend deployment & build process
- System configuration (vision, rubric, API)
- Security hardening procedures
- Monitoring & logging setup
- Backup & recovery procedures
- Troubleshooting common issues
- Rollback procedures

**Use this when:** Setting up a new environment or performing full deployment

---

### 2. [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) (11 KB)
**Printable checklist for deployment verification**

**Contents:**
- Pre-deployment checklist
- Development environment setup steps
- Production deployment steps
- Post-deployment verification
- Security hardening checklist
- Backup setup verification
- Monitoring configuration
- Final sign-off form

**Use this when:** Executing deployment and need to track progress

---

### 3. [DEPLOYMENT_TROUBLESHOOTING.md](DEPLOYMENT_TROUBLESHOOTING.md) (16 KB)
**Quick solutions for common deployment problems**

**Contents:**
- Backend issues (service, port, imports, API errors)
- Frontend issues (blank page, 404s, assets)
- Database issues (locked, not found, schema mismatch)
- Camera issues (not detected, poor quality, calibration)
- Network issues (access, CORS, connectivity)
- Performance issues (slow responses, memory)
- Service management (auto-start, restarts)
- Emergency procedures (reset, rollback, recovery)

**Use this when:** Encountering errors during or after deployment

---

## Supporting Documentation

### 4. [README.md](README.md) (16 KB)
**Project overview and quick start guide**

**Contents:**
- System overview
- Key features
- Quick start (5 minutes)
- Architecture diagrams
- API reference
- Frontend UI description
- Database schema
- Configuration examples
- Deployment summary
- Performance metrics

**Use this when:** Getting oriented with the project

---

### 5. [QUICK_START.md](QUICK_START.md) (14 KB)
**Fast setup procedures for common scenarios**

**Contents:**
- 5-minute Windows development setup
- 20-minute RDK X5 hardware setup
- Quick verification steps
- Common workflows (4 scenarios)
- Run modes reference
- Troubleshooting quick fixes
- Performance expectations
- API quick reference

**Use this when:** Need to get running quickly in dev or production

---

### 6. [RDK_X5_DEPLOYMENT_GUIDE.md](RDK_X5_DEPLOYMENT_GUIDE.md) (16 KB)
**Comprehensive RDK X5-specific deployment and operations**

**Contents:**
- System architecture
- Hardware setup procedures
- Installation steps
- Calibration workflow (3-step process)
- Operation modes (server, demo, calibrate, ros2)
- API endpoints (all 22 documented)
- ROS2 integration guide
- Troubleshooting RDK-specific issues

**Use this when:** Working specifically with RDK X5 hardware

---

### 7. [TECHNICAL_IMPLEMENTATION.md](TECHNICAL_IMPLEMENTATION.md) (16 KB)
**Deep technical details and architecture**

**Contents:**
- Component architecture (6 core modules)
- Data structures & models
- Data flow diagrams (4 pipelines)
- Algorithm details
- Performance optimization strategies
- Error handling patterns
- Testing strategy
- Deployment best practices
- Future enhancements

**Use this when:** Understanding system internals or planning modifications

---

### 8. [AI_MODEL_TRAINING_GUIDE.md](AI_MODEL_TRAINING_GUIDE.md) (28 KB)
**Hybrid Desktop-Training + Edge-Inference Architecture**

**Contents:**
- Complete AI workflow (capture → train → compile → deploy)
- Desktop GPU training with YOLOv8
- Horizon toolchain compilation (ONNX → BPU .bin)
- BPU inference with hobot_dnn (30-40 FPS)
- Performance benchmarks (20x speedup vs CPU)
- Retraining procedures
- Troubleshooting AI-specific issues

**Use this when:** Training custom defect detection models or optimizing AI performance

---

## Deployment Workflow

### Development Environment Setup

```
1. Read: QUICK_START.md (5-minute setup)
2. Follow: Development section
3. Verify: Checklist items
4. Debug: DEPLOYMENT_TROUBLESHOOTING.md if issues
```

**Time:** 5-15 minutes  
**Output:** Local dev environment with hot-reload

---

### Production Deployment

```
1. Review: DEPLOYMENT_INSTRUCTIONS.md (read Prerequisites)
2. Print: DEPLOYMENT_CHECKLIST.md
3. Execute: Production Deployment section
4. Verify: Post-Deployment Verification
5. Configure: Security Hardening
6. Setup: Monitoring & Backups
7. Test: Functional & Performance Tests
8. Debug: DEPLOYMENT_TROUBLESHOOTING.md if issues
```

**Time:** 2-4 hours  
**Output:** Production system on RDK X5

---

### Hardware-Specific Setup

```
1. Read: RDK_X5_DEPLOYMENT_GUIDE.md (System Architecture)
2. Follow: Hardware Setup section
3. Execute: Calibration Workflow
4. Configure: Operation Modes
5. Integrate: ROS2 (if needed)
```

**Time:** 1-2 hours (including calibration)  
**Output:** Calibrated camera system

---

### AI Model Training & Deployment

```
1. Read: AI_MODEL_TRAINING_GUIDE.md (Architecture Overview)
2. Collect: Weld images from RDK X5 (500+ per defect class)
3. Label: Using LabelImg or Roboflow
4. Train: YOLOv8 on Desktop GPU (1-2 hours)
5. Export: ONNX format
6. Compile: Horizon OE Docker toolchain (ONNX → .bin)
7. Deploy: Transfer .bin to RDK X5
8. Inference: Run on BPU at 30-40 FPS
```

**Time:** 4-8 hours (first time), 2-3 hours (retraining)  
**Output:** Compiled .bin model running on BPU at 40 FPS

---

## Quick Reference by Role

### System Administrator

**Primary Documents:**
1. DEPLOYMENT_INSTRUCTIONS.md (full procedures)
2. DEPLOYMENT_CHECKLIST.md (execution tracking)
3. DEPLOYMENT_TROUBLESHOOTING.md (problem solving)

**Key Tasks:**
- Server provisioning
- Service configuration
- Security hardening
- Backup setup
- Monitoring configuration

---

### DevOps Engineer

**Primary Documents:**
1. DEPLOYMENT_INSTRUCTIONS.md (automation source)
2. TECHNICAL_IMPLEMENTATION.md (architecture)
3. RDK_X5_DEPLOYMENT_GUIDE.md (hardware specifics)

**Key Tasks:**
- CI/CD pipeline setup
- Infrastructure as code
- Automated testing
- Performance monitoring
- Deployment automation

---

### ML Engineer / AI Specialist

**Primary Documents:**
1. AI_MODEL_TRAINING_GUIDE.md (complete AI pipeline)
2. TECHNICAL_IMPLEMENTATION.md (system integration)
3. DEPLOYMENT_TROUBLESHOOTING.md (AI-specific issues)

**Key Tasks:**
- Dataset collection and labeling
- Model training on desktop GPU
- Horizon toolchain compilation
- BPU performance optimization
- Model retraining and versioning

---

### Developer

**Primary Documents:**
1. QUICK_START.md (fast setup)
2. README.md (project overview)
3. TECHNICAL_IMPLEMENTATION.md (code architecture)

**Key Tasks:**
- Local development setup
- Feature development
- Testing
- Code contribution
- Documentation updates

---

### Hardware Technician

**Primary Documents:**
1. RDK_X5_DEPLOYMENT_GUIDE.md (hardware procedures)
2. DEPLOYMENT_CHECKLIST.md (verification)
3. DEPLOYMENT_TROUBLESHOOTING.md (camera issues)

**Key Tasks:**
- Camera installation
- Calibration execution
- Hardware troubleshooting
- Performance validation
- Maintenance

---

## Deployment Scenarios

### Scenario 1: Fresh Development Setup (Windows)

**Documents Needed:**
- QUICK_START.md
- README.md

**Steps:**
1. Install prerequisites (Python, Node.js)
2. Clone repository
3. Setup backend (virtual env, dependencies)
4. Setup frontend (npm install)
5. Run servers (Flask + Vite)
6. Verify in browser

**Time:** 5-10 minutes  
**Difficulty:** Easy

---

### Scenario 2: Production Deployment (RDK X5)

**Documents Needed:**
- DEPLOYMENT_INSTRUCTIONS.md
- DEPLOYMENT_CHECKLIST.md
- RDK_X5_DEPLOYMENT_GUIDE.md

**Steps:**
1. Prepare hardware (RDK X5 + camera)
2. Install system dependencies
3. Deploy application code
4. Configure camera & calibrate
5. Setup database
6. Create systemd services
7. Deploy frontend build
8. Configure nginx
9. Setup firewall
10. Verify deployment
11. Configure monitoring & backups

**Time:** 2-4 hours  
**Difficulty:** Moderate

---

### Scenario 3: Troubleshooting Deployment Issues

**Documents Needed:**
- DEPLOYMENT_TROUBLESHOOTING.md
- DEPLOYMENT_INSTRUCTIONS.md (for reference)

**Steps:**
1. Identify issue category (backend, frontend, database, camera, network)
2. Locate relevant section in troubleshooting guide
3. Follow diagnostic steps
4. Apply solution
5. Verify fix
6. Document for future reference

**Time:** 15 minutes - 2 hours  
**Difficulty:** Varies

---

### Scenario 4: System Update/Upgrade

**Documents Needed:**
- DEPLOYMENT_INSTRUCTIONS.md (Rollback section)
- DEPLOYMENT_CHECKLIST.md (verification)

**Steps:**
1. Backup current system
2. Test update in staging
3. Create rollback plan
4. Stop services
5. Update code/dependencies
6. Migrate database (if needed)
7. Rebuild frontend
8. Start services
9. Verify deployment
10. Monitor for issues

**Time:** 30 minutes - 1 hour  
**Difficulty:** Moderate

---

## File Size Summary

| Document | Size | Lines | Purpose |
|----------|------|-------|---------|
| DEPLOYMENT_INSTRUCTIONS.md | 29 KB | 900+ | Complete deployment procedures |
| DEPLOYMENT_CHECKLIST.md | 11 KB | 350+ | Verification checklist |
| DEPLOYMENT_TROUBLESHOOTING.md | 16 KB | 500+ | Problem solutions |
| AI_MODEL_TRAINING_GUIDE.md | 28 KB | 850+ | AI training & BPU deployment |
| README.md | 16 KB | 500+ | Project overview |
| QUICK_START.md | 14 KB | 500+ | Fast setup guide |
| RDK_X5_DEPLOYMENT_GUIDE.md | 16 KB | 450+ | Hardware-specific guide |
| TECHNICAL_IMPLEMENTATION.md | 16 KB | 650+ | Technical deep-dive |
| **TOTAL** | **146 KB** | **4,700+ lines** | **Complete documentation** |

---

## Documentation Maintenance

### Update Schedule

- **After code changes:** Update TECHNICAL_IMPLEMENTATION.md
- **After deployment process changes:** Update DEPLOYMENT_INSTRUCTIONS.md
- **After troubleshooting discoveries:** Update DEPLOYMENT_TROUBLESHOOTING.md
- **Quarterly:** Review all documentation for accuracy

### Version Control

All documentation is version-controlled in Git:
```bash
git log -- "*.md"
```

### Contributing

When updating documentation:
1. Create feature branch
2. Update relevant files
3. Test procedures if applicable
4. Submit pull request
5. Request review from team

---

## Support Channels

### Internal Support

1. **Check documentation first** (this index)
2. **Search existing issues** (GitHub)
3. **Review logs** (systemd, nginx)
4. **Ask team** (Slack/Teams channel)

### External Support

1. **RDK X5 Hardware:** [Horizon Robotics Support](https://github.com/HorizonRobotics/rdk)
2. **ROS2 Issues:** [ROS2 Documentation](https://docs.ros.org/)
3. **Flask/Python:** [Flask Documentation](https://flask.palletsprojects.com/)
4. **React/Vite:** [Vite Documentation](https://vitejs.dev/)

---

## Quick Commands Reference

### Check System Status
```bash
# All services
sudo systemctl status weld-evaluator nginx

# Application logs
sudo journalctl -u weld-evaluator -f

# Web server logs
sudo tail -f /var/log/nginx/access.log
```

### Restart Services
```bash
# Backend only
sudo systemctl restart weld-evaluator

# Frontend only (nginx)
sudo systemctl reload nginx

# Both
sudo systemctl restart weld-evaluator nginx
```

### Backup & Restore
```bash
# Backup
./scripts/backup_database.sh

# Restore
cp backups/production_YYYYMMDD_HHMMSS.db production.db
```

### Deployment Verification
```bash
# API test
curl http://localhost:5000/api/classes

# Frontend test
curl http://localhost/

# Camera test
ls /dev/video*

# Database test
sqlite3 production.db ".tables"
```

---

## Deployment Checklist Quick Summary

### Pre-Deployment
- [ ] Hardware ready
- [ ] Software prerequisites installed
- [ ] Network configured
- [ ] Documentation reviewed

### Deployment
- [ ] Code deployed
- [ ] Dependencies installed
- [ ] Database initialized
- [ ] Services configured
- [ ] Frontend built
- [ ] Nginx configured

### Post-Deployment
- [ ] All services running
- [ ] API responding
- [ ] Frontend accessible
- [ ] Camera working
- [ ] Monitoring active
- [ ] Backups configured

### Verification
- [ ] Functional tests passed
- [ ] Performance acceptable
- [ ] Security hardened
- [ ] Documentation updated

---

## Emergency Contacts

| Role | Responsibility | Contact |
|------|---------------|---------|
| System Admin | Infrastructure | ___________________ |
| DevOps Lead | Deployment | ___________________ |
| Tech Lead | Architecture | ___________________ |
| Hardware Tech | Camera/RDK | ___________________ |

---

**Last Updated:** December 17, 2025  
**Document Version:** 1.0  
**Maintained By:** DevOps Team

*For questions or corrections, open an issue on GitHub*
