#!/usr/bin/env python3
"""Release management script.

Automated version release workflow including:
- Version number updates
- Changelog updates
- Git tag creation
- Build and release
"""

import argparse
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple


class ReleaseManager:
    """Release manager."""
    
    def __init__(self, project_root: Path):
        self.project_root = project_root
        self.version_file = project_root / "libfrankapy" / "__init__.py"
        self.changelog_file = project_root / "CHANGELOG.md"
        
    def get_current_version(self) -> str:
        """Get current version number."""
        content = self.version_file.read_text(encoding="utf-8")
        match = re.search(r'__version__\s*=\s*["\']([^"\']*)["\'']', content)
        if not match:
            raise ValueError("Cannot find version number")
        return match.group(1)
    
    def update_version(self, new_version: str) -> None:
        """Update version number."""
        content = self.version_file.read_text(encoding="utf-8")
        new_content = re.sub(
            r'(__version__\s*=\s*["\'])([^"\']*)(["\'])',
            f'\\g<1>{new_version}\\g<3>',
            content
        )
        self.version_file.write_text(new_content, encoding="utf-8")
        print(f"✅ Version updated to: {new_version}")
    
    def validate_version(self, version: str) -> bool:
        """Validate version number format."""
        pattern = r'^\d+\.\d+\.\d+(?:-[a-zA-Z0-9]+(?:\.[a-zA-Z0-9]+)*)?$'
        return bool(re.match(pattern, version))
    
    def get_next_version(self, current: str, bump_type: str) -> str:
        """Calculate next version number."""
        parts = current.split('.')
        if len(parts) != 3:
            raise ValueError(f"Invalid version format: {current}")
        
        major, minor, patch = map(int, parts)
        
        if bump_type == "major":
            return f"{major + 1}.0.0"
        elif bump_type == "minor":
            return f"{major}.{minor + 1}.0"
        elif bump_type == "patch":
            return f"{major}.{minor}.{patch + 1}"
        else:
            raise ValueError(f"Invalid version type: {bump_type}")
    
    def update_changelog(self, version: str, changes: Optional[List[str]] = None) -> None:
        """Update changelog."""
        if not self.changelog_file.exists():
            print("⚠️ CHANGELOG.md does not exist, skipping update")
            return
        
        content = self.changelog_file.read_text(encoding="utf-8")
        today = datetime.now().strftime("%Y-%m-%d")
        
        # Find [Unreleased] section
        unreleased_pattern = r'## \[Unreleased\]\s*\n(.*?)(?=\n## |\n---|\Z)'
        match = re.search(unreleased_pattern, content, re.DOTALL)
        
        if match:
            unreleased_content = match.group(1).strip()
            
            # Create new release section
            release_section = f"## [{version}] - {today}\n\n{unreleased_content}\n\n"
            
            # Reset unreleased section
            new_unreleased = """## [Unreleased]

### Added
- None

### Changed
- None

### Fixed
- None

### Removed
- None

### Security
- None

"""
            
            # Replace content
            new_content = content.replace(
                match.group(0),
                new_unreleased + release_section
            )
            
            self.changelog_file.write_text(new_content, encoding="utf-8")
            print(f"✅ CHANGELOG.md updated, added version {version}")
        else:
            print("⚠️ Cannot find [Unreleased] section, please update CHANGELOG.md manually")
    
    def run_command(self, cmd: List[str], check: bool = True) -> subprocess.CompletedProcess:
        """Run command."""
        print(f"🔧 Running command: {' '.join(cmd)}")
        result = subprocess.run(cmd, cwd=self.project_root, capture_output=True, text=True)
        
        if check and result.returncode != 0:
            print(f"❌ Command failed: {' '.join(cmd)}")
            print(f"Error output: {result.stderr}")
            sys.exit(1)
        
        return result
    
    def check_git_status(self) -> bool:
        """Check Git status."""
        result = self.run_command(["git", "status", "--porcelain"], check=False)
        if result.stdout.strip():
            print("❌ Working directory is not clean, please commit all changes first")
            print(result.stdout)
            return False
        return True
    
    def create_git_tag(self, version: str) -> None:
        """Create Git tag."""
        tag_name = f"v{version}"
        
        # Check if tag already exists
        result = self.run_command(["git", "tag", "-l", tag_name], check=False)
        if result.stdout.strip():
            print(f"⚠️ Tag {tag_name} already exists")
            return
        
        # Create tag
        self.run_command(["git", "add", "."])
        self.run_command(["git", "commit", "-m", f"chore: bump version to {version}"])
        self.run_command(["git", "tag", "-a", tag_name, "-m", f"Release {version}"])
        
        print(f"✅ Git tag created: {tag_name}")
    
    def build_package(self) -> None:
        """Build package."""
        print("🔨 Building package...")
        
        # Clean old build files
        self.run_command(["rm", "-rf", "build", "dist", "*.egg-info"], check=False)
        
        # Build
        self.run_command([sys.executable, "-m", "build"])
        
        print("✅ Package build completed")
    
    def run_tests(self) -> None:
        """Run tests."""
        print("🧪 Running tests...")
        self.run_command([sys.executable, "-m", "pytest", "tests/", "-v"])
        print("✅ Tests passed")
    
    def check_package(self) -> None:
        """Check package."""
        print("🔍 Checking package...")
        self.run_command([sys.executable, "-m", "twine", "check", "dist/*"])
        print("✅ Package check passed")
    
    def release(self, version: str, bump_type: Optional[str] = None, 
                dry_run: bool = False, skip_tests: bool = False) -> None:
        """Execute release workflow."""
        print(f"🚀 Starting release workflow: {version}")
        
        if dry_run:
            print("🔍 This is a dry run, no actual operations will be performed")
        
        # Check Git status
        if not dry_run and not self.check_git_status():
            return
        
        # Validate version number
        if not self.validate_version(version):
            print(f"❌ Invalid version number: {version}")
            return
        
        current_version = self.get_current_version()
        print(f"📋 Current version: {current_version}")
        print(f"📋 Target version: {version}")
        
        if not dry_run:
            # Run tests
            if not skip_tests:
                self.run_tests()
            
            # Update version
            self.update_version(version)
            
            # Update changelog
            self.update_changelog(version)
            
            # Build package
            self.build_package()
            
            # Check package
            self.check_package()
            
            # Create Git tag
            self.create_git_tag(version)
            
            print(f"🎉 Release {version} completed!")
            print("📝 Next steps:")
            print(f"   1. Push to remote repository: git push origin main --tags")
            print(f"   2. Publish to PyPI: python -m twine upload dist/*")
            print(f"   3. Create Release on GitHub")
        else:
            print("✅ Dry run completed, all checks passed")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="LibFrankaPy release management tool")
    parser.add_argument("version", nargs="?", help="Target version number")
    parser.add_argument("--bump", choices=["major", "minor", "patch"], 
                       help="Automatically calculate version number")
    parser.add_argument("--dry-run", action="store_true", help="Dry run, no actual operations")
    parser.add_argument("--skip-tests", action="store_true", help="Skip tests")
    parser.add_argument("--current", action="store_true", help="Show current version number")
    
    args = parser.parse_args()
    
    # Find project root directory
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    manager = ReleaseManager(project_root)
    
    if args.current:
        print(f"Current version: {manager.get_current_version()}")
        return
    
    # Determine target version
    if args.version:
        target_version = args.version
    elif args.bump:
        current = manager.get_current_version()
        target_version = manager.get_next_version(current, args.bump)
    else:
        parser.print_help()
        return
    
    # Execute release
    manager.release(
        version=target_version,
        bump_type=args.bump,
        dry_run=args.dry_run,
        skip_tests=args.skip_tests
    )


if __name__ == "__main__":
    main()