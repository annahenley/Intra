using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Geometry.Intersect;
using IntraLattice.Properties;
using Grasshopper;
using IntraLattice.CORE.Data;
using IntraLattice.CORE.Components;
using IntraLattice.CORE.Helpers;
using TriangulationComponents;



// Summary:     This component generates a pseudo-random Voronoi lattice trimmed to the shape of the design space.
// ==========================================================================================================================
// Details:     - Design space may be a Mesh, Brep or Solid Surface.
//              - Orientation plane does not need to be centered at any particular location
// ==========================================================================================================================
// Issues:      - Mesh design spaces with many coplanar faces are prone to failure.. issue with Rhino's Mesh.isInside method
// ==========================================================================================================================
// Author(s):   Aidan Kurtz (http://aidankurtz.com), Anna Henley


namespace IntraLattice.CORE.Components
{
    public class Voronoi : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Voronoi class.
        /// </summary>
        public Voronoi()
            : base("Voronoi", "Voro",
                "Generates a pseudo-random 3D Voronoi lattice within a design space",
                "IntraLattice", "Frame")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("Design Space", "DS", "Design Space (Brep or Mesh)", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Orientation Plane", "Plane", "Lattice orientation plane", GH_ParamAccess.item, Plane.WorldXY); // default is XY-plane
            pManager.AddNumberParameter("Tolerance", "Tol", "Smallest allowed strut length", GH_ParamAccess.item, 0.2);
            pManager.AddBooleanParameter("Strict tolerance", "Strict", "Specifies if we use a strict tolerance.", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Number of Points", "NumPts", "Number of source points to generate", GH_ParamAccess.item);
        }


        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter ("Struts", "Struts", "lattice brep network", GH_ParamAccess.list);
            pManager.AddBrepParameter("Control Lattice", "Control", "pre-screwing around voronoi structure", GH_ParamAccess.tree);
        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //1. Retrieve and validate data.

            GeometryBase designSpace = null;
            Plane orientationPlane = Plane.Unset;
            double numPts = 0;
            bool strictlyIn = false;

            if (!DA.GetData(0, ref designSpace)) { return; }
            if (!DA.GetData(1, ref orientationPlane)) { return; }
            if (!DA.GetData(3, ref strictlyIn)) { return; }
            if (!DA.GetData(4, ref numPts)) { return; }

            if (!designSpace.IsValid) { return; }
            if (!orientationPlane.IsValid) { return; }
            if (numPts == 0) { return; }

            // 2. Validate the design space
            int spaceType = FrameTools.ValidateSpace(ref designSpace);
            if (spaceType == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Design space must be a closed Brep, Mesh or Surface");
                return;
            }

            double tol = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;

            // 3. Compute oriented bounding box and its boundaries in each direction
            Box bBox = new Box();
            designSpace.GetBoundingBox(orientationPlane, out bBox);
            Point3d[] bBoxCorners = bBox.GetCorners();

            //    Set basePlane based on the bounding box
            Plane basePlane = new Plane(bBoxCorners[0], bBoxCorners[1], bBoxCorners[3]);

            //   Get box bounds and store in array
            Point3d xMin = bBox.PointAt(0, 0, 0);
            Point3d xMax = bBox.PointAt(1, 0, 0);
            Point3d yMin = bBox.PointAt(0, 0, 0);
            Point3d yMax = bBox.PointAt(0, 1, 0);
            Point3d zMin = bBox.PointAt(0, 0, 0);
            Point3d zMax = bBox.PointAt(0, 0, 1);

            double[] Bounds = new double[6] { xMin.X, xMax.X, yMin.Y, yMax.Y, zMin.Z, zMax.Z };

            //Prepare to generate random points
            Random rand = new Random();
            List<Point3d> pts = new List<Point3d>();
            Point3d point = new Point3d();

            //Generate random points within X,Y,Z boundaries

            for (int i = 0; i <= numPts; i++)
            {
                point.X = Bounds[0] + (Bounds[1] - Bounds[0]) * rand.NextDouble();
                point.Y = Bounds[2] + (Bounds[3] - Bounds[2]) * rand.NextDouble();
                point.Z = Bounds[4] + (Bounds[5] - Bounds[4]) * rand.NextDouble();
                pts.Add(point);
            }

            //Cull points outside designSpace
            List<Point3d> cull = new List<Point3d>();
            Brep brep = new Brep();
            if (Brep.TryConvertBrep(designSpace) == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Design space must be a closed Brep, Mesh or Surface");
                return;
            }
            GH_Convert.ToBrep_Primary(designSpace, ref brep);
            for (int i = pts.Count - 1; i >= 0; i--)
            {
                if (brep.IsPointInside(pts[i], 0.0001, false) == false)
                {
                    cull.Add(pts[i]);
                    pts.RemoveAt(i);
                }
            }

            //Compute average of points
            double sumX = new double();
            double sumY = new double();
            double sumZ = new double();
            for (int i = pts.Count - 1; i >= 0; i--)
            {
                sumX += pts[i].X;
                sumY += pts[i].Y;
                sumZ += pts[i].Z;
            }
            double avgX = sumX / pts.Count;
            double avgY = sumY / pts.Count;
            double avgZ = sumZ / pts.Count;
            Point3d centre = new Point3d(avgX, avgY, avgZ);

            //Scale points up 1.2x, using average as centre
            List<Point3d> scalePts = new List<Point3d>();
            foreach (Point3d pt in pts)
            {
                pt.Transform(Rhino.Geometry.Transform.Scale(centre, 1.2)); scalePts.Add(pt);
            }
            BoundingBox scaledBbox = new BoundingBox(scalePts);

            //Construct preliminary 3D Voronoi from original points and scaled-up box
            GH_Brep brepi = new GH_Brep();
            List<Brep> breps = new List<Brep>();
            DataTree<object> prelimVoro = new DataTree<object>();
            prelimVoro = FrameTools.ConstructVoro(pts, scaledBbox);
            /*Curve[] intcrvs = new Curve[prelimVoro.Branches.Count];
            Point3d[] intpts = new Point3d[prelimVoro.Branches.Count];
            List<Curve[]> frame = new List<Curve[]>();*/

            //Compute intersection of Voronoi and original designSpace
            /*for (int i = 0; i < prelimVoro.Branches.Count; i++)
            {
                for (int j = 0; j < prelimVoro.Branch(i).Count; j++)
                {

                    brepi = (GH_Brep)prelimVoro.Branch(i)[j];
                    breps.Add(brepi.Value);
                    Rhino.Geometry.Intersect.Intersection.BrepBrep(brepi.Value, brep, 0.01, out intcrvs, out intpts);
                    frame.Add(intcrvs);
                    foreach (Curve entry in frame)
                    {
                        entry = (GH_Curve)entry;
                    }
                }



                //Will use to scale designSpace


                //Voronoi method call *****Now a separate method but kept in here for a bit just in case*****
                //Create the component, treating "pts" as an input
                /*var cs = new TriangulationComponents.Component_VolumeVoronoi();
                //add the points list (input 0)
                var points = cs.Params.Input[0] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
                points.PersistentData.ClearData();
                for (int i = 0; i < (int)pts.Count; i++)
                {
                    points.PersistentData.Append(new GH_Point(pts[i]));
                }

                cs.ExpireSolution(true);


                //add to a dummy document so we can read outputs
                var doc = new Grasshopper.Kernel.GH_Document();
                doc.AddObject(cs, false);


                //Initialize temporary brep storage
                //List<> temp = new List <object>();

                cs.Params.Output[0].CollectData();
                cs.Params.Output[1].CollectData();
                DataTree<object> tempA = new DataTree<object>();
                DataTree<object> tempB = new DataTree<object>();

                //Loop thru points, storing each branch in a temporary tree
                for (int j = pts.Count - 1; j >= 0; j--)
                {
                    GH_Path pth = new GH_Path(j);
                    //object branchJ = cs.Params.Output[0].VolatileData.get_Branch(0)[j], pth;
                    tempA.Add(cs.Params.Output[0].VolatileData.get_Branch(0)[j], pth);  //this function returns a branch of a data tree
                }
            
            
                DataTree<object> rawStruts = tempA;

                //remove that component
                doc.RemoveObject(cs.Attributes, false);

                //Set output data*/

                //DA.SetData(0, FrameTools.SolidIntersect(prelimVoro, (Brep)designSpace));
                //DA.SetDataTree(0, FrameTools.ConstructVoro(pts, scaledBbox));
               // LineCurve temp = new LineCurve();
               // Curve tempCurve = (Curve)temp;
                /*List<Curve> crvList = new List<Curve>();
                for (int k = 0; k < frame.Count; k++) {
                    for (int j = 0; j < frame[k].Length; j++)
                        {
                           GH_Convert.ToCurve_Secondary(frame[k][j], ref tempCurve);
                            crvList.Add(tempCurve);
                        }
                }*/
                
               DA.SetDataList(0,FrameTools.SolidIntersect(prelimVoro, brep));
               DA.SetDataTree(1, FrameTools.ConstructVoro(pts, scaledBbox));
              
               
           
        }
        


        /// <summary>
        /// Sets the exposure of the component (i.e. the toolbar panel it is in)
        /// </summary>
        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.tertiary;
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{90c33a0e-18a3-456d-b01b-6f71153302c3}"); }
        }
    }
}