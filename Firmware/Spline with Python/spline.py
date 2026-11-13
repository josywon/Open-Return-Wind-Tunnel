import adsk.core
import adsk.fusion
import traceback


# Dimensions in millimetres
LENGTH_MM = 300.0
LARGE_SIDE_MM = 240.0
SMALL_SIDE_MM = 120.0
SPLINE_POINTS = 61  # More points give Fusion a closer fit to the equation.


def smoothstep5(t):
    """Quintic transition from 0 to 1 with zero slope at both ends."""
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        if not design:
            ui.messageBox('Open a Fusion design before running this script.')
            return

        root = design.rootComponent
        sketches = root.sketches
        xy_plane = root.xYConstructionPlane

        # Convert millimetres to the design's internal length units (cm).
        mm_to_internal = 0.1
        length = LENGTH_MM * mm_to_internal
        large_half = (LARGE_SIDE_MM / 2.0) * mm_to_internal
        small_half = (SMALL_SIDE_MM / 2.0) * mm_to_internal

        # Profiles are on XY planes at each end. X is tunnel length;
        # Y and Z describe the square cross-section.
        start_plane_input = root.constructionPlanes.createInput()
        start_plane_input.setByOffset(
            xy_plane, adsk.core.ValueInput.createByReal(0.0)
        )
        start_plane = root.constructionPlanes.add(start_plane_input)

        end_plane_input = root.constructionPlanes.createInput()
        end_plane_input.setByOffset(
            xy_plane, adsk.core.ValueInput.createByReal(length)
        )
        end_plane = root.constructionPlanes.add(end_plane_input)

        start_profile = sketches.add(start_plane)
        start_profile.name = 'Opening profile - 240 mm square'
        end_profile = sketches.add(end_plane)
        end_profile.name = 'Exit profile - 120 mm square'

        def draw_square(sketch, half_size):
            lines = sketch.sketchCurves.sketchLines
            corners = [
                adsk.core.Point3D.create(-half_size, -half_size, 0),
                adsk.core.Point3D.create( half_size, -half_size, 0),
                adsk.core.Point3D.create( half_size,  half_size, 0),
                adsk.core.Point3D.create(-half_size,  half_size, 0),
            ]
            for i in range(4):
                lines.addByTwoPoints(corners[i], corners[(i + 1) % 4])

        draw_square(start_profile, large_half)
        draw_square(end_profile, small_half)

        # Create a 3D sketch for four corresponding corner splines.
        # At each x, the corner coordinates follow the Desmos half-height
        # h = 120 - 60*(10t^3 - 15t^4 + 6t^5), with t = x/300.
        path_sketch = sketches.add(xy_plane)
        path_sketch.name = 'Four smooth corner transition splines'
        path_sketch.is3D = True
        spline_collection = path_sketch.sketchCurves.sketchFittedSplines

        for y_sign in (-1, 1):
            for z_sign in (-1, 1):
                points = adsk.core.ObjectCollection.create()

                for i in range(SPLINE_POINTS):
                    t = i / float(SPLINE_POINTS - 1)
                    x = length * t
                    half = large_half - (large_half - small_half) * smoothstep5(t)
                    points.add(adsk.core.Point3D.create(
                        x,
                        y_sign * half,
                        z_sign * half
                    ))

                spline_collection.add(points)

        ui.messageBox(
            'Created the two square profiles and four corner splines. '
            'Use the profiles as loft sections and the splines as guide rails.'
        )

    except:
        if ui:
            ui.messageBox('Script failed:\n{}'.format(traceback.format_exc()))

