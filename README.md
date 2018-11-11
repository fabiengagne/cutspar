# cutspar
Cutspar modifies a pair of .dat airfoil files to create the contoured notches required for applying spar caps on a wing panel foam core. The spar caps follow the curvature of the airfoil, that is, they're not necessarily and flat pieces of precured carbon fiber. The software deals with synchronizing the features of the notches so it can be correctly cut on the CNC hot-wire cutter.

cutspar -I rootinput.dat -i tipinput.dat -O rootoutput.dat -o tipoutput.dat -C chord;efwd;eaft;ethk;ifwd;iaft;ithk -c chord;efwd;eaft;ethk;ifwd;iaft;ithk [-x] [-t roottwist;tiptwist[;pivotpoint]

-I rootinput.dat : input airfoil file name at root of panel (airfoil .dat)

-i tipinput.dat  : input airfoil file name at tip of panel (airfoil .dat)

-O rootoutput.dat : output airfoil file names for root

-o tipoutput.dat : output airfoil file names for tip

-x Exit and re-enter the leading edge. Allows the LE to cool down before cutting the other half of the profile.

-t roottwist;tiptwist[;pivotpoint] : Rotates the airfoil (washout) around a pivot point. Negative angle = LE down. pivotpoint is in percent from LE, and negative means to rotate at the intrados skin, positive at the extrados skin (default=-70).

-C specifications at root (see below for the mandatory 7 parameters)

-c specifications at tip

  chord : chord of at this station (root or tip)

  efwd : extrados spar forward edge
  
  eaft : extrados spar aft edge
  
  ethk : extrados spar thickness (notch depth)
  
  ifwd : intrados spar forward edge
  
  iaft : intrados spar aft edge
  
  ithk : intrados spar thickness (notch depth)
  

efwd, eaft, ifwd and iaft are distances relative to the LE at their respective\nstation.

Example:

cutspar -I ../../SynerJ-90.dat -i ../../SynerJ-80.dat -C 221.5;31.62;81.62;2;32.62;82.62;2 -c 196.5;20.54;70.54;2;21.54;71.54;2 -O Mid2-root.dat -o Mid2-tip.dat -x -t -0.20;-0.35;-70

The software will cope with spar of different widths, position or thickness at each panel ends, or even top & bottom, keeping the points in the .dat files correctly synchronized.

# Using cutspar with GMFC
Using the .dat files created by cutspar on GMFC is a piece of cake. Just make sure to use the "complex" mode instead of the "wing" mode. This mode is preferred because in "wing" mode, GMFC makes some assumptions about the .dat file being stricly an airfoil, which is no longer true once cutspar creates creates notches on the surface.

However, using the "complex" mode has drawbacks : we loose very useful functions like the X-leading edge and points densification. Thankfully, cutspar supplements with similar functions.


# To compile
Compiled on Windows 10 using Code::Blocks v17.12 http://www.codeblocks.org/ 
This POSIX compliant code should compile just as fine on Linux or Apple.
