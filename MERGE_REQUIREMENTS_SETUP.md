# GitHub Merge Requirements Setup

Complete guide to enforce test passage before PRs can be merged.

---

## 🔧 Step 1: Verify GitHub Actions Workflow

Your workflow is already configured at `.github/workflows/unit-tests.yml`

The workflow defines 3 jobs that produce status checks:
```yaml
jobs:
  algorithm-tests:      # Produces status: "algorithm-tests"
  rosbag-tests:         # Produces status: "rosbag-tests"
  gazebo-tests:         # Produces status: "gazebo-tests"
```

---

## 📋 Step 2: Configure Branch Protection Rules (GitHub UI)

### **For main branch (protect against accidental merges):**

1. Go to your repository
2. **Settings** → **Branches** (left sidebar)
3. Click **Add rule**

### **Configuration:**

**Branch name pattern:** `main`

**✅ Enable these options:**

- [x] **Require a pull request before merging**
  - [x] Require approvals (set to 1-2 reviewers)
  - [x] Dismiss stale pull request approvals when new commits are pushed
  - [x] Require review from code owners

- [x] **Require status checks to pass before merging**
  - [x] Require branches to be up to date before merging
  - Select required status checks:
    - `algorithm-tests` ✅
    - `rosbag-tests` ✅
    - `gazebo-tests` ✅

- [x] **Require code scanning results to be reviewed before merging**
  - (Optional: if using CodeQL)

- [x] **Include administrators**
  - (So even admins can't bypass the checks)

- [x] **Restrict who can push to matching branches**
  - (Optional: limit to main maintainers)

---

## 🛡️ Step 3: CLI Setup (Automated)

If you prefer to set this up via command line:

```bash
# Install GitHub CLI if not already done
brew install gh

# Authenticate
gh auth login

# Create branch protection rule
gh api repos/McGillFormulaElectric/MFE-Driverless-V1/branches/main/protection \
  -X POST \
  -f required_status_checks='{"strict":true,"contexts":["algorithm-tests","rosbag-tests","gazebo-tests"]}' \
  -f required_pull_request_reviews='{"required_approving_review_count":1,"dismiss_stale_reviews":true}' \
  -f allow_force_pushes=false \
  -f allow_deletions=false
```

---

## 🚀 Step 4: Test the Setup

### **Try merging a PR without passing tests:**

1. Create a test PR with a failing test
2. Attempt to merge → **Should be blocked** with message:
   ```
   ❌ algorithm-tests (required) — Some checks were not successful
   ```

### **Merge a PR with passing tests:**

1. Create a PR with all tests passing
2. Attempt to merge → **Should be allowed**

---

## 📊 Current Workflow Status Checks

Your workflow creates these status checks that are now mergeable gates:

| Status Check | Triggered By | Passes When |
|--------------|--------------|-------------|
| `algorithm-tests` | Changes to core modules | `pytest src/mfe_*/test/test_*.py` passes |
| `rosbag-tests` | Changes to sensor/perception | rosbag validation passes |
| `gazebo-tests` | Changes to control/planning | Gazebo mission simulation passes |

---

## ⚙️ Step 5: Configure for Feature Branches (Optional)

If you want to also protect feature branches:

```bash
# Create rule for all branches matching pattern
gh api repos/McGillFormulaElectric/MFE-Driverless-V1/branches/*/protection \
  -X POST \
  -f pattern='feature/*' \
  -f required_status_checks='{"strict":true,"contexts":["algorithm-tests"]}' \
  -f required_pull_request_reviews='{"required_approving_review_count":0}'
```

This allows feature branch PRs to be merged once tests pass (no review required).

---

## 📝 Step 6: View Current Protection Rules

```bash
# View all branch protection rules
gh api repos/McGillFormulaElectric/MFE-Driverless-V1/branches/main/protection

# View rules for a specific branch
gh api repos/McGillFormulaElectric/MFE-Driverless-V1/protected-branches
```

---

## 🔄 Step 7: Update Workflow (If Needed)

To add more status checks or modify job names, edit `.github/workflows/unit-tests.yml`:

```yaml
jobs:
  algorithm-tests:
    runs-on: ubuntu-latest
    # This job name becomes: "algorithm-tests" status check
    
  rosbag-tests:
    runs-on: ubuntu-latest
    # This job name becomes: "rosbag-tests" status check
    
  gazebo-tests:
    runs-on: ubuntu-latest
    # This job name becomes: "gazebo-tests" status check
```

**Important:** The status check name must exactly match the job name in the workflow file.

---

## 🎯 Best Practices

### **1. Require All Tests**
```
✅ All three status checks (algorithm, rosbag, gazebo) required
✅ Require branches to be up to date
✅ Dismiss stale reviews on new commits
```

### **2. Set Review Requirements**
```
✅ Require 1-2 reviewers (depending on team size)
✅ Require code owner review (if using CODEOWNERS file)
✅ Disable approval by author
```

### **3. Set Merge Strategy**
```
✅ Allow squash merge (clean history)
✅ Allow rebase merge (preserve commits)
❌ Require squash merge (too restrictive)
```

### **4. Monitor Merge Queue**
```bash
# View pending PRs blocked by checks
gh pr list --state open --label "blocked" --json title,statusCheckRollup
```

---

## 📋 CODEOWNERS File (Optional)

Create `.github/CODEOWNERS` to require specific reviewers for certain files:

```
# Control and safety-critical code requires 2 approvals
ros2/src/mfe_control/ @neil-handle @supervisor-handle

# Perception code requires perception team
ros2/src/mfe_perception/ @perception-lead

# Planning requires planning team  
ros2/src/mfe_path_planning/ @planning-lead

# All PRs need at least one general reviewer
* @repo-owner
```

---

## 🚨 Troubleshooting

### **"Status check is required but has no status"**
→ Workflow job name doesn't match status check name
→ Fix: Ensure workflow job name = required status check

### **"Branch is out of date with the base branch"**
→ PR needs to be rebased on latest main
→ Fix: Enable "Require branches to be up to date before merging"

### **"Can't merge - tests still running"**
→ Wait for GitHub Actions to complete
→ View progress at Actions tab

### **"Dismiss stale reviews not working"**
→ Need to re-review after new commits
→ This is expected behavior to catch new issues

---

## 🔐 Enforcement Levels

| Level | Setup | When to Use |
|-------|-------|------------|
| **Low** | Status checks only | Early dev, experimental branches |
| **Medium** | Status checks + 1 review | Feature branches, active development |
| **High** | All checks + 2 reviews + codeowners | main, release branches |

---

## 📈 Recommended Configuration for MFE-Driverless-V1

```
Branch: main
├─ Required status checks: ALL 3 (algorithm, rosbag, gazebo)
├─ Require 1 approval minimum
├─ Dismiss stale reviews on new commits
├─ Require up-to-date before merge
├─ Include administrators
└─ No force push allowed

Branch: feature/*
├─ Required status checks: algorithm-tests only
├─ No approval required (team trust)
├─ Require up-to-date before merge
└─ Allow force push (WIP branches)
```

---

## ✅ Verification Checklist

After setup:

- [ ] Create a test PR with failing test
- [ ] Attempt to merge → **Should be blocked** ❌
- [ ] Fix the test, push update
- [ ] Merge button becomes available → **Should allow** ✅
- [ ] Check merge commit shows "Merged by Github"
- [ ] All 3 status checks appear as green checkmarks

---

## 🔗 References

- [GitHub Docs: Branch protection rules](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/managing-a-branch-protection-rule)
- [GitHub Actions: Workflow syntax](https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions)
- [GitHub CLI: Branch protection](https://cli.github.com/manual/gh_api)

---

**Next Steps:**
1. Go to Settings → Branches
2. Add protection rule for main
3. Select the 3 status checks
4. Save and test
