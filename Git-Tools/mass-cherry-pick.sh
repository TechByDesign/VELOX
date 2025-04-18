'''
Usage: mass-cherry-pick.sh [-f <file_path>] <commit_hash> <branch1> [branch2] [branch3] ...

Description:
    This script either:
    1. Cherry-picks a specific file from a commit into multiple branches
    2. Synchronizes multiple branches with the first branch provided

Arguments:
    -f <file_path>   Optional: Path to specific file to cherry-pick
    <commit_hash>    The hash of the commit containing the desired changes
    <branch1>        Source branch (required)
    [branch2]...     Target branches to update (optional)

Examples:
    # Cherry-pick specific file to branches
    ./mass-cherry-pick.sh -f src/main.js abc123 feature1 feature2

    # Sync branches with feature1
    ./mass-cherry-pick.sh abc123 feature1 feature2 develop staging

Note:
    - Ensure you have a clean working directory before running
    - Returns to original branch after completion
    - Skips branches that don't exist or can't be checked out
'''

#!/bin/bash

# Initialize variables
FILE_PATH=""
COMMIT_HASH=""
BRANCHES=()

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -f)
            FILE_PATH="$2"
            shift 2
            ;;
        *)
            if [ -z "$COMMIT_HASH" ]; then
                COMMIT_HASH=$1
            else
                BRANCHES+=("$1")
            fi
            shift
            ;;
    esac
done

# Validate inputs
if [ ${#BRANCHES[@]} -lt 1 ]; then
    echo "Usage: $0 [-f <file_path>] <commit_hash> <branch1> [branch2] [branch3] ..."
    exit 1
fi

# Validate commit hash
if ! git cat-file -e $COMMIT_HASH^{commit} 2>/dev/null; then
    echo "Error: Invalid commit hash"
    exit 1
fi

# Store current branch
CURRENT_BRANCH=$(git symbolic-ref --short HEAD)
SOURCE_BRANCH=${BRANCHES[0]}

# Remove source branch from array
BRANCHES=("${BRANCHES[@]:1}")

# Loop through target branches
for branch in "${BRANCHES[@]}"; do
    echo "Processing branch: $branch"
    
    # Check if branch exists
    if ! git show-ref --verify --quiet refs/heads/$branch; then
        echo "Warning: Branch $branch doesn't exist. Skipping."
        continue
    fi
    
    # Checkout branch
    if ! git checkout $branch; then
        echo "Error: Could not checkout $branch"
        continue
    fi
    
    if [ -n "$FILE_PATH" ]; then
        # Cherry-pick specific file
        git checkout $COMMIT_HASH -- "$FILE_PATH"
        
        # Check if there are changes to commit
        if git diff --quiet; then
            echo "No changes to commit in $branch"
            continue
        fi
        
        git commit -m "Cherry-picked $FILE_PATH from commit $COMMIT_HASH"
    else
        # Sync entire branch
        echo "Syncing $branch with $SOURCE_BRANCH"
        if ! git merge $SOURCE_BRANCH --no-edit; then
            echo "Error: Merge conflicts in $branch. Please resolve manually."
            continue
        fi
    fi
    
    echo "Successfully updated $branch"
done

# Return to original branch
git checkout $CURRENT_BRANCH

echo "Done!"
