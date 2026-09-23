#!/bin/bash
# Quick start: Set up branch protection rules via CLI

REPO="McGillFormulaElectric/MFE-Driverless-V1"
BRANCH="main"

echo "Setting up branch protection for: $REPO/$BRANCH"
echo ""

# Install gh CLI if needed
if ! command -v gh &> /dev/null; then
    echo "Installing GitHub CLI..."
    brew install gh
fi

# Authenticate if needed
echo "Authenticating with GitHub..."
gh auth login --web || true

echo ""
echo "Creating branch protection rule..."

# Create protection rule for main branch
gh api repos/$REPO/branches/$BRANCH/protection \
  -X PUT \
  --input - << 'CONFIG'
{
  "required_status_checks": {
    "strict": true,
    "contexts": [
      "algorithm-tests",
      "rosbag-tests",
      "gazebo-tests"
    ]
  },
  "required_pull_request_reviews": {
    "required_approving_review_count": 1,
    "dismiss_stale_reviews": true,
    "require_code_owner_reviews": false
  },
  "enforce_admins": true,
  "allow_force_pushes": false,
  "allow_deletions": false,
  "restrict_dismissals": false
}
CONFIG

echo ""
echo "✅ Branch protection rule created!"
echo ""
echo "Verify with:"
echo "  gh api repos/$REPO/branches/$BRANCH/protection"
echo ""
echo "Next: Go to GitHub Settings → Branches to verify"
