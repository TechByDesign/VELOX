# CHANGELOG

## [Unreleased]
### Fixed
- **Module Reloading Issues**
  - Fixed module reloading to ensure code changes take effect without restarting Blender
  - Added proper error handling for module imports

- **Support System**
  - Fixed `SupportType` enum handling in support detection
  - Improved error messages for invalid support types
  - Fixed support detection when no vertex is selected

### Known Issues
- Force visualization shows green spheres on all edges regardless of force magnitude
- Force application cylinders appear on incorrect vertices
- Force distribution through the structure is not working as expected

## Version 0.1.8 - 2025-05-27
### Features
- **Enhanced Support System**
  - Added Support class for better support management
  - Support for different support types (FIXED, PINNED, ROLLER)
  - Improved auto-detection of support vertices
  - Visual indicators for different support types

### Improvements
- **Chassis Analysis**
  - Default support type set to FIXED for chassis analysis
  - Better distribution of auto-detected supports
  - Support for up to 4 wheel positions

### Bug Fixes
- Fixed support visualization for different support types
- Improved error handling in support detection

## Version 0.1.7 - 2025-05-25
### Features
- Enhanced vertex selection system
  - Added proper BMesh handling for vertex selection
  - Implemented automatic support detection
  - Added visual feedback for selected points

### Bug Fixes
- Fixed BMesh resource management
  - Added proper cleanup with try/finally blocks
  - Added validation for BMesh existence
  - Fixed mode switching issues

### Development Changes
- Restructured development roadmap
  - Prioritized FEA implementation and supports over material properties
  - Added detailed accuracy verification features
  - Reorganized phases to focus on core FEA implementation first

### Lessons Learned
1. **Resource Management**
   - Always clean up resources (BMesh, collections, etc.)
   - Use try/finally blocks for resource cleanup
   - Validate resource existence before use

2. **State Management**
   - Track mode changes carefully
   - Validate state transitions
   - Clean up temporary objects

3. **Development Process**
   - Test changes in isolation
   - Document assumptions
   - Update documentation with changes

4. **Roadmap Planning**
   - Focus on core accuracy features first
   - Prioritize FEA implementation over UI/UX
   - Add detailed validation requirements early

## Version 0.1.6 - 2025-05-24

## Version 0.1.5 - 2025-05-24
### Features
- Enhanced force calculation system
  - Added customizable force components (X, Y, Z)
  - Implemented force vector projection for more realistic force distribution
  - Added force magnitude multiplier
  - Improved force propagation through structure

### Bug Fixes
- Fixed edge force calculation to properly use selected vertex
  - Added proper edge normalization
  - Added edge existence validation
  - Fixed force distribution logic

## Version 0.1.4 - 2025-05-24
### Bug Fixes
- Fixed cylinder scaling and rotation issues
  - Properly handle zero-length edges
  - Fixed scale calculations for better visualization
  - Ensured proper alignment of cylinders with edges
- Fixed text label positioning
  - Corrected midpoint calculation for text placement
  - Ensured text follows edge orientation
- Improved non-destructive workflow
  - No longer modifies original mesh
  - All visualization is done on separate objects

## Version 0.1.3 - 2025-05-24
### Bug Fixes
- Fixed force vector handling in analysis
  - Added proper conversion between Blender Vector and NumPy array types
  - Ensured force vector is properly passed to analysis function
- Fixed visualization errors
  - Updated cylinder creation to use proper 3-tuple coordinates instead of vector addition
  - Added proper vertex position handling from indices
  - Fixed force application edge highlighting
  - Added support for edge length calculation
  - Added support_indices parameter to visualize_results for proper support visualization

### Changes
- Improved error handling in visualization
  - Added proper cleanup of previous visualization objects
  - Added visibility management for visualization collection

## Version 0.1.2 - 2025-05-24
### Features
- Added clear_analysis function to properly clean up previous results
  - Removes all force visualization objects
  - Cleans up materials
  - Removes forces collection

### Bug Fixes
- Fixed force vector handling in main execution loop
  - Added proper Vector creation with correct force magnitude
  - Ensured force vector is properly passed to analysis function

## Version 0.1.1 - 2025-05-24
### Features
- Added professional visualization system
  - Color-coded cylinders for force visualization
  - Force magnitude labels
  - Highlighted force application edges
  - Support point markers
- Added user-friendly terminal interface
  - Clear menu options
  - Detailed error messages
  - Step-by-step guidance

### Bug Fixes
- Added proper error handling for:
  - No object selected
  - Invalid menu choices
  - Analysis errors
  - Collection cleanup errors

## Version 0.1.0 - Initial Release
### Features
- Basic structural analysis functionality
- Support detection system
- Force application system
- Basic visualization capabilities

---

Note: This changelog follows the [Keep a Changelog](https://keepachangelog.com/) format and uses [Semantic Versioning](https://semver.org/).
