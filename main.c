/*
cutspar  by Fabien Gagne Nov 2018  released under the GNU GPLv3

cutspar allows to create a notch top & bottom of two airfoils .dat files for spar
caps. It makes sure that the features of the notches are synchronized in both .dat
files so it can be correctly cut with a CNC hot-wire cutter.

History:
 v1.0   Initial release
 v1.1   Added -x option

Compiled on Windows 10 using Code::Blocks v17.12
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <libgen.h>

#define VERSION "v1.1"

#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

typedef struct
{
    double x, y;
} coord_t;

typedef struct
{
    coord_t fwd;  // spar forward edge (.x is specified by the user)
    coord_t aft;    // spar aft edge (.x is specified by the user)

    int    fwd_idx, aft_idx; // same as above, but as indexes into stationdef_t.p[]

    double thick;       // thickness of spar, specified by the user
} spar_t;


// root & tip stations definition
typedef struct
{
    char *i_fname;   // loaded from this filename
    char *o_fname;   // output written to this filename

    char desc[81];  // first line of the .dat file (text description)
    double chord;

    spar_t spar[2];  // 0=extrados, 1=intrados

    int n;              // number of points in p[]
    coord_t p[1000];    // array of points from the .dat, scaled to chord

    int on;             // number of points in op[]
    coord_t op[1000];   // array of computed points

    int le;             // index into p[] of the leading edge
} stationdef_t;


// TRUE if x is within [a,b] bounds
int within(double x, double a, double b)
{
    if (a<=b && a<=x && x<=b)
        return 1;
    if (a>=b && b<=x && x<=a)
        return 1;
    return 0;
}


double gett(double t, coord_t *p0, coord_t *p1)
{
    double alpha = 0.5f; // 0.5 = centripetal catmull-rom spline
    double a = pow((p1->x - p0->x), 2.0f) + pow((p1->y - p0->y), 2.0f);
    double b = pow(a, 0.5f);
    double c = pow(b, alpha);

    return (c + t);
}

// Centripetal Catmull-Rom interpolation. Interpolates any point between p[1] and p[2], given 4 control points.
double CatmullRom ( double x, coord_t *p )
{
    double t0, t1, t2, t3;
    coord_t a1, a2, a3, b1, b2, c;
    double t;

    t0 = 0;
    t1 = gett(t0, &p[0], &p[1]);
    t2 = gett(t1, &p[1], &p[2]);
    t3 = gett(t2, &p[2], &p[3]);

    t = (x - p[1].x)/(p[2].x - p[1].x) * (t2 - t1) + t1;

    // make sure x is within the interval p[1].x to p[2].x
    if ( !within(x, p[1].x, p[2].x) )
    {
        printf("CatmullRom received x=%lf that is not within the interval [%lf;%lf]\n", x, p[1].x, p[2].x);
        exit(EXIT_FAILURE);
    }

    a1.x = (t1-t)/(t1-t0)*p[0].x + (t-t0)/(t1-t0)*p[1].x;
    a1.y = (t1-t)/(t1-t0)*p[0].y + (t-t0)/(t1-t0)*p[1].y;

    a2.x = (t2-t)/(t2-t1)*p[1].x + (t-t1)/(t2-t1)*p[2].x;
    a2.y = (t2-t)/(t2-t1)*p[1].y + (t-t1)/(t2-t1)*p[2].y;

    a3.x = (t3-t)/(t3-t2)*p[2].x + (t-t2)/(t3-t2)*p[3].x;
    a3.y = (t3-t)/(t3-t2)*p[2].y + (t-t2)/(t3-t2)*p[3].y;

    b1.x = (t2-t)/(t2-t0)*a1.x + (t-t0)/(t2-t0)*a2.x;
    b1.y = (t2-t)/(t2-t0)*a1.y + (t-t0)/(t2-t0)*a2.y;

    b2.x = (t3-t)/(t3-t1)*a2.x + (t-t1)/(t3-t1)*a3.x;
    b2.y = (t3-t)/(t3-t1)*a2.y + (t-t1)/(t3-t1)*a3.y;

    c.x = (t2-t)/(t2-t1)*b1.x + (t-t1)/(t2-t1)*b2.x;
    c.y = (t2-t)/(t2-t1)*b1.y + (t-t1)/(t2-t1)*b2.y;

    return c.y;
}



int loaddatfile ( stationdef_t *station )
{
    FILE *f;
    double x, y, y1=0;

    printf ("Loading %s ", station->i_fname);

    f = fopen(station->i_fname,"r");
    if ( f == NULL )
    {
        printf("%s: %s\n", station->i_fname, strerror(errno));
        goto err;
    }
    if ( fscanf(f,"%80s\n", &station->desc[0]) != 1 )
    {
        printf("%s: file format\n", station->i_fname);
        goto err;
    }

    for (;station->n < sizeof(station->p)/sizeof(station->p[0]);)
    {
        if ( fscanf (f, "%lf %lf\n", &x, &y ) == 2 )
        {
            // find the leading edge
            if ( x < 0.1f && ((y>=0 && y1<0) || (y<=0 && y1>0)))
                station->le = station->n;
            y1 = y;

            // scale to chord
            station->p[station->n].x = x * station->chord;
            station->p[station->n].y = y * station->chord;
            station->n++;
        }
        else break;
    }

    fclose (f);



    printf("%d points\n", station->n);
    return 0;

err:
    fclose(f);
    return -1;
}

int savedatfile ( stationdef_t *station )
{
    FILE *f;

    printf("Saving %s ", station->o_fname);

    f = fopen(station->o_fname, "w");
    if(f == NULL)
    {

        printf("%s : %s\n", station->o_fname, strerror(errno));
        return -1;
    }
    fprintf(f, "%s-%s\n", basename(station->o_fname), station->desc);
    for(int i=0; i<station->on; i++)
    {
        fprintf(f, "%9.06f   %9.06f\n", station->op[i].x/station->chord, station->op[i].y/station->chord );
    }
    printf("%d points\n",station->on);
    return fclose(f);
}

// given an arbitrary x and a face (extrados or intrados), interpolate a y with the catmull-rom
double find_y ( double x, int face, stationdef_t *station)
{
    // search the index of the points where 'x' lies in-between two points
    for (int i=2; i<station->n-1; i++)
    {
        int dos = (station->p[i].y >= 0) ? 0:1; // extrados/intrados

        if ( dos == face && (within(x, station->p[i-1].x, station->p[i].x)) )
            return CatmullRom(x, &station->p[i-2]);
    }
    printf("oops %f %d\n", x, face);
    return 0;
}


void findspar ( stationdef_t *station )
{
    int i;
    int dos;

    // Locate the beginning & end of the extrados and intrados spar to find their
    // respective index in the list of points station->p[].
    for (i=0; i<station->n; i++)
    {
        dos = (station->p[i].y >= 0) ? 0:1; // extrados/intrados

        if ( station->p[i].x <= station->spar[dos].aft.x && (!station->spar[dos].aft_idx || station->p[i].x > station->p[station->spar[dos].aft_idx].x))
            station->spar[dos].aft_idx = i;
        if ( station->p[i].x >= station->spar[dos].fwd.x && (!station->spar[dos].fwd_idx || station->p[i].x < station->p[station->spar[dos].fwd_idx].x))
            station->spar[dos].fwd_idx = i;
    }

    // compute the y coordinates of the spar edges using the catmull-rom interpolation
    for (dos=0; dos<2; dos++)
    {
        station->spar[dos].fwd.y = find_y(station->spar[dos].fwd.x, dos, station);
        station->spar[dos].aft.y = find_y(station->spar[dos].aft.x, dos, station);
    }
}


        // sign of a integer
int isignnum ( int val )
{
    return ( (0<val) - (val<0) );
}

// sign of a double
int fsignnum ( double val )
{
    return ( (0<val) - (val<0) );
}

double integral(double *func, int n)
{
    double sum;
    int i;

    for(sum=0, i=0;i<n;i++)
        sum += func[i];
    return sum;
}

// Find the scaling factor that make it such that the integral(func) = target value
double scaledx(double *func, int n, double target )
{
    return target / integral(func,n);
}


void usage (char *pname)
{
    printf("cutspar allows to create a notch top & bottom of two airfoils .dat files for\n");
    printf("spar caps. It makes sure that the features of the notches are synchronized in\n");
    printf("both .dat files so it can be correctly cut with a CNC hot-wire cutter.\n\n");

    printf("%s -I rootinput.dat -i tipinput.dat -O rootoutput.dat -o tipoutput.dat -C chord;efwd;eaft;ethk;ifwd;iaft;ithk -c chord;efwd;eaft;ethk;ifwd;iaft;ithk [-x]\n\n", pname);

    printf("-I rootinput.dat : input airfoil file name at root of panel (airfoil .dat)\n");
    printf("-i tipinput.dat  : input airfoil file name at tip of panel (airfoil .dat)\n");
    printf("-O rootoutput.dat : output airfoil file names for root\n");
    printf("-o tipoutput.dat : output airfoil file names for tip\n");
    printf("-x Exit and re-enter the leading edge. Allows the LE to cool down before cutting\n   the other half of the profile.\n");
    printf("-C specifications at root (see below for the mandatory 7 parameters)\n");
    printf("-c specifications at tip\n");
    printf("  chord : chord of at this station (root or tip)\n");

    printf("  efwd : extrados spar forward edge\n");
    printf("  eaft : extrados spar aft edge\n");
    printf("  ethk : extrados spar thickness (notch depth)\n");
    printf("  ifwd : intrados spar forward edge\n");
    printf("  iaft : intrados spar aft edge\n");
    printf("  ithk : intrados spar thickness (notch depth)\n\n");

    printf("efwd, eaft, ifwd and iaft are distances relative to the LE at their respective\nstation.\n\n");

    printf("Example:\n");
    printf("cutspar -I ../../SynerJ-90.dat -i ../../SynerJ-80.dat -O Mid2-root.dat -o Mid2-tip.dat -C 221.5;31.62;81.62;1.2;32.62;82.62;1.2 -c 196.5;20.54;70.54;1.1;21.54;71.54;1.1\n\n");

    printf("cutspar %s %s\n", VERSION, __DATE__);
    printf("Fabien Gagne, 2018\n");
}

int main(int argc, char *argv[])
{
    int opt;
    int ns = 11;  // smooth the stretching/compression on this many points, before and after the spar.

    struct {  // Exit and re-enter the leading edge
        int     active; // 0=off, 1=on
        double  dx, dy; // width & height, in mm
    } xle = { 0,            // default=off
          10.0, 10.0
    };

    stationdef_t root;
    stationdef_t tip;

    coord_t prootfwd[2][50], ptipfwd[2][50];  // new serie of points forward of the spar
    coord_t prootaft[2][50], ptipaft[2][50];  // new serie of points aft of the spar
    coord_t prootspar[2][100], ptipspar[2][100]; // new serie of point at the spar
    int nspar[2]; // number of points of the spar (0=extrados, 1=intrados), that is number of elements in prootspar[] and ptipspar[]

    int sfwd[2], saft[2]; // indexes where the smoothing ends, before and after the spar


    memset (&root, 0, sizeof(root));
    memset (&tip, 0, sizeof(tip));


    if (argc <= 1)
    {
        usage(argv[0]);
        exit(-1);
    }
    while ((opt = getopt(argc, argv, "ho:O:i:I:c:C:x")) != -1 )
    {
        int n;

        switch (opt)
        {
            case 'I': root.i_fname = optarg; break;
            case 'i': tip.i_fname = optarg;  break;

            case 'O': root.o_fname = optarg; break;
            case 'o': tip.o_fname = optarg;  break;

            case 'C':   // root specifications
                n = sscanf(optarg, "%lf;%lf;%lf;%lf;%lf;%lf;%lf\n", &root.chord, &root.spar[0].fwd.x, &root.spar[0].aft.x, &root.spar[0].thick, &root.spar[1].fwd.x, &root.spar[1].aft.x, &root.spar[1].thick );
                if ( n != 7 || root.spar[0].fwd.x > root.spar[0].aft.x || root.spar[0].aft.x > root.chord || root.spar[1].fwd.x > root.spar[1].aft.x || root.spar[1].aft.x > root.chord)
                {
                    usage(argv[0]);
                    exit(-1);
                }
                break;

            case 'c': // tip specifications
                n = sscanf(optarg, "%lf;%lf;%lf;%lf;%lf;%lf;%lf\n", &tip.chord, &tip.spar[0].fwd.x, &tip.spar[0].aft.x, &tip.spar[0].thick, &tip.spar[1].fwd.x, &tip.spar[1].aft.x, &tip.spar[1].thick );
                if ( n != 7 || tip.spar[0].fwd.x > tip.spar[0].aft.x || tip.spar[0].aft.x > tip.chord || tip.spar[1].fwd.x > tip.spar[1].aft.x || tip.spar[1].aft.x > tip.chord )
                {
                    usage(argv[0]);
                    exit(-1);
                }
                break;

            case 'x':
                xle.active = 1;
                break;

            case 'h':
                usage(argv[0]);
                exit(-1);

        }
    }

    if ( root.i_fname == NULL || tip.i_fname == NULL )
    {
        printf ("Both -I and -i options must be specified.\n");
        exit(-1);
    }

    if ( root.o_fname == NULL || tip.o_fname == NULL )
    {
        printf ("Both -O and -o options must be specified.\n");
        exit(-1);
    }

    if ( root.chord <= 0 || tip.chord <= 0 )
    {
        printf ("Both -C and -c options must be specified.\n");
        exit(-1);
    }

    // Load .dat files and scale to respective chords
    if ( loaddatfile (&root) == -1 || loaddatfile (&tip) == -1 )
        exit(-1);

    if ( root.n != tip.n )
    {
        printf("Both dat files need to have the same number of points.\n");
        exit(-1);
    }

    // Locate the beginning & end of the extrados and intrados spar
    findspar(&root);
    findspar(&tip);

    if (root.le != tip.le)
        printf("Warning: LE is not positionned at same point number in both files. This may not\nwork. If they differ by 3 points or less, it may be okay.\n");

    // Adjust the smoothing window so that it not be any closer than 3 points from the leading edge
    ns = MIN(abs(root.spar[0].fwd_idx - root.le), abs(root.spar[1].fwd_idx - root.le) );
    ns = MIN(ns, abs(tip.spar[0].fwd_idx - root.le) );
    ns = MIN(ns, abs(tip.spar[1].fwd_idx - root.le) );
    ns -= 3;
    ns = MIN(ns,48);

    for (int dos=0; dos<2; dos++)
    {
        double raft_dx, taft_dx;
        double rspar_dx, tspar_dx;
        int sign;
        int fwd, aft;
        double rdx[50], tdx[50], rf, tf;


        sign = isignnum(root.spar[dos].aft_idx - root.spar[dos].fwd_idx); // points ordered clockwise/counterclockwise
        if ( sign != isignnum(tip.spar[dos].aft_idx - tip.spar[dos].fwd_idx) || sign==0 )
        {
            printf("Coordinates within both airfoils are not ordered the same in the files (clockwise/counter clockwise)\n");
            exit(-1);
        }

        fwd = sign>0 ? MAX(root.spar[dos].fwd_idx, tip.spar[dos].fwd_idx) : MIN(root.spar[dos].fwd_idx, tip.spar[dos].fwd_idx);
        aft = sign>0 ? MAX(root.spar[dos].aft_idx, tip.spar[dos].aft_idx) : MAX(root.spar[dos].aft_idx, tip.spar[dos].aft_idx);

        // sfwd and saft are indexes where the smoothing ends, before and after the spar
        sfwd[dos] = fwd - sign*ns;
        saft[dos] = aft + sign*ns;

        // For both root and tip, compute "ns" new points forward and aft of the spar, including the point at the edge of the spar
        // aft : evenly spaced in x
        raft_dx = root.spar[dos].aft.x - root.p[saft[dos]].x;
        taft_dx = tip.spar[dos].aft.x - tip.p[saft[dos]].x;

        // forward : get the spacing from the original airfoil, and expand.
        for (int i=0;i<=ns;i++)
        {
            rdx[i] = root.p[sfwd[dos]+i*sign].x - root.p[sfwd[dos]+i*sign+sign].x; // derivative
            tdx[i] =  tip.p[sfwd[dos]+i*sign].x -  tip.p[sfwd[dos]+i*sign+sign].x;
        }
        rf = scaledx(rdx, ns, root.spar[dos].fwd.x - root.p[sfwd[dos]].x );
        tf = scaledx(tdx, ns,  tip.spar[dos].fwd.x -  tip.p[sfwd[dos]].x );


        for(int i=0; i<=ns; i++)
        {
            // Forward of spar : 'ns+1' points spaced in x proportionally similar to the original airfoil
            prootfwd[dos][i].x = root.p[sfwd[dos]].x + integral(rdx,i)*rf;
            ptipfwd[dos][i].x  =  tip.p[sfwd[dos]].x + integral(tdx,i)*tf;

            // Aft of spar: 'ns+1' points spaced equally in x
            prootaft[dos][i].x = root.p[saft[dos]].x + i*raft_dx/(double)ns;
            ptipaft[dos][i].x  =  tip.p[saft[dos]].x + i*taft_dx/(double)ns;

            // interpolate their 'y' coordinates
            prootfwd[dos][i].y = find_y(prootfwd[dos][i].x, dos, &root);
            prootaft[dos][i].y = find_y(prootaft[dos][i].x, dos, &root);
            ptipfwd[dos][i].y  = find_y(ptipfwd[dos][i].x, dos, &tip);
            ptipaft[dos][i].y  = find_y(ptipaft[dos][i].x, dos, &tip);
        }

        // compute the points for the spar itself
        nspar[dos] = 2 + MAX ( abs(root.spar[dos].aft_idx - root.spar[dos].fwd_idx), abs(tip.spar[dos].aft_idx - tip.spar[dos].fwd_idx));
        rspar_dx = root.spar[dos].fwd.x - root.spar[dos].aft.x;
        tspar_dx =  tip.spar[dos].fwd.x -  tip.spar[dos].aft.x;

        for (int i=0; i<=nspar[dos]; i++)
        {
            // 'n+1' points spaced equally in x
            prootspar[dos][i].x = root.spar[dos].aft.x + i*rspar_dx/nspar[dos];
            ptipspar[dos][i].x  =  tip.spar[dos].aft.x + i*tspar_dx/nspar[dos];

            // interpolate their 'y' coordinates
            prootspar[dos][i].y = find_y(prootspar[dos][i].x, dos, &root) + (dos==0?-1:1)*root.spar[dos].thick;
            ptipspar[dos][i].y  = find_y(ptipspar[dos][i].x, dos, &tip)   + (dos==0?-1:1)*root.spar[dos].thick;
        }
    }

    // Merge the unchanged part of the root and tip airfoils with the new
    // ones. The new airfoils will have a few more points. root and tip will
    // end up with the same number of points.
    for (int i=0; i<root.n; )
    {
        for (int dos=0;dos<2;dos++)
        {
            if ( i == sfwd[dos] ) // where the smoothing begins forward of the spar
            {
                // insert the new points forward of spar
                for(int j=0;j<=ns;j++)
                {
                    root.op[root.on++] = prootfwd[dos][j];
                    tip.op[tip.on++]   = ptipfwd[dos][j];
                }
                // insert the spar
                for(int j=nspar[dos]; j>=0;j--)
                {
                    root.op[root.on++] = prootspar[dos][j];
                    tip.op[tip.on++]   = ptipspar[dos][j];
                }
                // insert the new points aft of spar
                for(int j=ns;j>=0;j--)
                {
                    root.op[root.on++] = prootaft[dos][j];
                    tip.op[tip.on++]   = ptipaft[dos][j];
                }
                i = saft[dos]; // skip to aft of the spar
                goto skip;
            }

            else if (i == saft[dos]) // where the smoothing begins aft of the spar
            {
                // insert the new points aft of spar
                for(int j=0;j<=ns;j++)
                {
                    root.op[root.on++] = prootaft[dos][j];
                    tip.op[tip.on++]   = ptipaft[dos][j];
                }
                // insert the spar
                for(int j=0; j<=nspar[dos];j++)
                {
                    root.op[root.on++] = prootspar[dos][j];
                    tip.op[tip.on++]   = ptipspar[dos][j];
                }
                // insert the new points forward of spar
                for(int j=ns;j>=0;j--)
                {
                    root.op[root.on++] = prootfwd[dos][j];
                    tip.op[tip.on++]   = ptipfwd[dos][j];
                }

                i = sfwd[dos];
                goto skip;
            }
        }

        // Its just a regular point in between our zones of interest, copy.
        root.op[root.on++] = root.p[i];
        tip.op[tip.on++] = tip.p[i];

        // At the LE, insert the points for the exit & re-entry at the leading edge, if requested.
        if ( xle.active && i == root.le)
        {
            int dir = -fsignnum(root.p[i-1].y -  root.p[i].y); // 'dir' will be negative when going down

            /*  --      The following will create a shape like this in front of the LE, outside the profile.
               |   \    We exit the LE at 45 degrees, and re-enter at 45 degrees.
               |   /
                --
            */

            root.op[root.on].x   = -xle.dx/2 + root.p[i].x;
            root.op[root.on++].y = dir * xle.dy + root.p[i].y;
            tip.op[tip.on].x     = -xle.dx/2 + tip.p[i].x;
            tip.op[tip.on++].y   = dir * xle.dy + tip.p[i].y;

            root.op[root.on].x   = -xle.dx + root.p[i].x;
            root.op[root.on++].y = dir * xle.dy + root.p[i].y;
            tip.op[tip.on].x     = -xle.dx + tip.p[i].x;
            tip.op[tip.on++].y   = dir * xle.dy + tip.p[i].y;

            root.op[root.on].x   = -xle.dx + root.p[i].x;
            root.op[root.on++].y = -dir * xle.dy + root.p[i].y;
            tip.op[tip.on].x     = -xle.dx + tip.p[i].x;
            tip.op[tip.on++].y   = -dir * xle.dy + tip.p[i].y;

            root.op[root.on].x   = -xle.dx/2 + root.p[i].x;
            root.op[root.on++].y =  -dir * xle.dy + root.p[i].y;
            tip.op[tip.on].x     = -xle.dx/2 + tip.p[i].x;
            tip.op[tip.on++].y   = -dir * xle.dy + tip.p[i].y;

            root.op[root.on++] = root.p[i]; // then return to the LE point
            tip.op[tip.on++]   = tip.p[i];
        }

    skip:
        i++;
    }

    // Save the output files
    savedatfile(&root);
    savedatfile(&tip);

    return 0;
}
