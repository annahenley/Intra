﻿using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Types;
using Rhino.DocObjects;
using IntraLattice.Properties;
using System.Drawing;
using Grasshopper.Kernel.Expressions;
using IntraLattice.CORE.Helpers;

// This component generates various gradient expressions, for the HeterogenGradient component. 
// ===============================================================================
// Ideally, this should be replaced by a simple value list, this isn't very elegant.
// ===============================================================================
// Author(s):   Aidan Kurtz (http://aidankurtz.com)

namespace IntraLattice.CORE.MeshModule
{
    public class PresetGradientComponent : GH_Component
    {
        GH_Document GrasshopperDocument;
        IGH_Component Component;

        /// <summary>
        /// Initializes a new instance of the PresetGradientComponent class.
        /// </summary>
        public PresetGradientComponent()
            : base("Preset Gradient", "PresetGradient",
                "Generates gradient string (i.e. a spatial math expression)",
                "IntraLattice", "Utils")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Gradient Type", "Type", "Selection of gradient types", GH_ParamAccess.item, 0);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Gradient String", "Grad", "The spatial gradient as an expression string", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 0. Setup input
            Component = this;
            GrasshopperDocument = this.OnPingDocument();
            //    Generate default input menu
            if (Component.Params.Input[0].SourceCount == 0) InputTools.GradientSelect(ref Component, ref GrasshopperDocument, 0, 11);

            // 1. Retrieve input
            int gradientType = 0;
            if (!DA.GetData(0, ref gradientType)) { return; }

            // 2. Initialize 
            string mathString = null;

            // 3. Define gradients here
            // Assume unitized domain ( 0<x<1 , 0<y<1, 0<z<1), where radius values range from minRadius (mathString=0) to maxRadius (mathString=1)
            // Based on this assumption, the actual values are scaled to the size of the bounding box of the lattice
            switch (gradientType)
            {
                case 0:     // Linear (X)
                    mathString = "Abs(x)";
                    break;
                case 1:     // Linear (Y)
                    mathString = "Abs(y)";
                    break;
                case 2:     // Linear (Z)
                    mathString = "Abs(z)";
                    break;
                case 3:     // Centered (X)
                    mathString = "Abs(2*x-1)";
                    break;
                case 4:     // Centered (Y)
                    mathString = "Abs(2*y-1)";
                    break;
                case 5:     // Centered (Z)
                    mathString = "Abs(2*z-1)";
                    break;
                case 6:     // Cylindrical (X)
                    mathString = "Sqrt(Abs(2*y-1)^2 + Abs(2*z-1)^2)/Sqrt(2)";
                    break;
                case 7:     // Cylindrical (Y)
                    mathString = "Sqrt(Abs(2*x-1)^2 + Abs(2*z-1)^2)/Sqrt(2)";
                    break;
                case 8:     // Cylindrical (Z)
                    mathString = "Sqrt(Abs(2*x-1)^2 + Abs(2*y-1)^2)/Sqrt(2)";
                    break;
                case 9:     // Spherical
                    mathString = "Sqrt(Abs(2*x-1)^2 + Abs(2*y-1)^2 + Abs(2*z-1)^2)/Sqrt(3)";
                    break;
                // If you add a new gradient, don't forget to add it in the value list (GradientSelect method)
            }

            // Output report
            DA.SetData(0, mathString);

        }

        /// <summary>
        /// Sets the exposure of the component (i.e. the toolbar panel it is in)
        /// </summary>
        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.secondary;
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.presetGradient;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{6a4e5dcf-5d72-49fc-a543-c2465b14eb86}"); }
        }
    }
}