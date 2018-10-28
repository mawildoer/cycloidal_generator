#Author - mawildoer
#Description - cycloidal drive generator

import adsk.core, adsk.fusion, adsk.cam, traceback
import math

def getPoint(t, R, Rr, E, N):
    #psi = -math.atan(math.sin((1 - N) * theta) / ((R / (E * N)) - math.cos((1 - N) * theta)))
    #x = R * math.cos(theta) - Rr * math.cos(theta - psi) - E * math.cos(N * theta)
    #y =  - R * math.sin(theta) + Rr * math.sin(theta - psi) + E * math.cos(N * theta)
    psi = math.atan2(math.sin((1-N)*t), ((R/(E*N))-math.cos((1-N)*t)))

    x = (R*math.cos(t))-(Rr*math.cos(t+psi))-(E*math.cos(N*t))
    y = (-R*math.sin(t))+(Rr*math.sin(t+psi))+(E*math.sin(N*t))
    #x = (10*math.cos(t))-(1.5*math.cos(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))-(0.75*math.cos(10*t))
    #y = (-10*math.sin(t))+(1.5*math.sin(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))+(0.75*math.sin(10*t))
    return (x,y)

def getDist(xa, ya, xb, yb):
    return math.sqrt((xa-xb)**2 + (ya-yb)**2)

def run(context):
    ui = None

    try:

        app = adsk.core.Application.get()
        ui  = app.userInterface
        des = adsk.fusion.Design.cast(app.activeProduct)

        #####   CHANGE THE VALUES BELOW FOR DIMENSIONS #####
        rotorThickness = 1 #rotor thickness(cm)
        housingThickness = 2 * rotorThickness
        R = 5 #rotor radius (cm)
        N = 50 #number of rollers
        ####################################################



        #other constants based on the original inputs
        housing_cir = 2 * R * math.pi
        Rr = housing_cir / (4 * N) #roller radius
        E = 0.5 * Rr #eccentricity
        maxDist = 0.25 * Rr #maximum allowed distance between points
        minDist = 0.5 * maxDist #the minimum allowed distance between points

        root = des.rootComponent

        rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        rotor = rotorOcc.component
        rotor.name = 'rotor'

        sk = rotor.sketches.add(root.xYConstructionPlane)

        points = adsk.core.ObjectCollection.create()

        #ui.messageBox('Ratio will be ' + 1/N)

        (xs, ys) = getPoint(0, R, Rr, E, N)
        points.add(adsk.core.Point3D.create(xs,ys,0))

        et = 2 * math.pi / (N-1)
        (xe, ye) = getPoint(et, R, Rr, E, N)
        x = xs
        y = ys
        dist = 0
        ct = 0
        dt = math.pi / N
        numPoints = 0

        while ((math.sqrt((x-xe)**2 + (y-ye)**2) > maxDist or ct < et/2) and ct < et): #close enough to the end to call it, but over half way
        #while (ct < et/80): #close enough to the end to call it, but over half way
            (xt, yt) = getPoint(ct+dt, R, Rr, E, N)
            dist = getDist(x, y, xt, yt)

            ddt = dt/2
            lastTooBig = False
            lastTooSmall = False

            while (dist > maxDist or dist < minDist):
                if (dist > maxDist):
                    if (lastTooSmall):
                        ddt /= 2

                    lastTooSmall = False
                    lastTooBig = True

                    if (ddt > dt/2):
                        ddt = dt/2

                    dt -= ddt

                elif (dist < minDist):
                    if (lastTooBig):
                        ddt /= 2

                    lastTooSmall = True
                    lastTooBig = False
                    dt += ddt


                (xt, yt) = getPoint(ct+dt, R, Rr, E, N)
                dist = getDist(x, y, xt, yt)

            x = xt
            y = yt
            points.add(adsk.core.Point3D.create(x,y,0))
            numPoints += 1
            ct += dt

        points.add(adsk.core.Point3D.create(xe,ye,0))
        crv = sk.sketchCurves.sketchFittedSplines.add(points)

        lines = sk.sketchCurves.sketchLines;
        line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), crv.startSketchPoint)
        line2 = lines.addByTwoPoints(line1.startSketchPoint, crv.endSketchPoint)

        prof = sk.profiles.item(0)
        distance = adsk.core.ValueInput.createByReal(rotorThickness)

        # Get extrude features
        extrudes = rotor.features.extrudeFeatures
        extrude1 = extrudes.addSimple(prof, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # Get the extrusion body
        body1 = extrude1.bodies.item(0)
        body1.name = "rotor"

        inputEntites = adsk.core.ObjectCollection.create()
        inputEntites.add(body1)

        # Get Z axis for circular pattern
        zAxis = rotor.zConstructionAxis

        # Create the input for circular pattern
        circularFeats = rotor.features.circularPatternFeatures
        circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

        # Set the quantity of the elements
        circularFeatInput.quantity = adsk.core.ValueInput.createByReal(N-1)

        # Set the angle of the circular pattern
        circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

        # Set symmetry of the circular pattern
        circularFeatInput.isSymmetric = True

        # Create the circular pattern
        circularFeat = circularFeats.add(circularFeatInput)

        ToolBodies = adsk.core.ObjectCollection.create()
        for b in circularFeat.bodies:
            ToolBodies.add(b)

        combineInput = rotor.features.combineFeatures.createInput(body1, ToolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isNewComponent = False

        rotor.features.combineFeatures.add(combineInput)

        #Offset the rotor to make the shaft rotat concentric with origin
        transform = rotorOcc.transform
        transform.translation = adsk.core.Vector3D.create(E, 0, 0)
        rotorOcc.transform = transform
        des.snapshots.add()

        housingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        housing = housingOcc.component
        housing.name = 'housing'

        #add a sketch so rotor clearance is obvious
        sketches = housing.sketches
        rotorClearanceSketch = sketches.add(root.xYConstructionPlane)
        sketchCircles = rotorClearanceSketch.sketchCurves.sketchCircles
        centerPoint = adsk.core.Point3D.create(0, 0, 0)
        sketchCircles.addByCenterRadius(centerPoint, R)

        #add rollers
        rollerSketch = sketches.add(root.xYConstructionPlane)
        sketchCircles = rollerSketch.sketchCurves.sketchCircles
        centerPoint = adsk.core.Point3D.create(R, 0, 0)
        sketchCircles.addByCenterRadius(centerPoint, Rr)

        rollerProfile = rollerSketch.profiles.item(0)
        distance = adsk.core.ValueInput.createByReal(housingThickness)
        rollerExtrudes = housing.features.extrudeFeatures.addSimple(rollerProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Get the extrusion body
        roller = rollerExtrudes.bodies.item(0)
        roller.name = "roller"

        inputEntites = adsk.core.ObjectCollection.create()
        inputEntites.add(roller)

        # Create the input for circular pattern
        circularFeats = housing.features.circularPatternFeatures
        zAxis = housing.zConstructionAxis
        circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

        # Set the quantity of the elements
        circularFeatInput.quantity = adsk.core.ValueInput.createByReal(N)

        # Set the angle of the circular pattern
        circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

        # Set symmetry of the circular pattern
        circularFeatInput.isSymmetric = True

        # Create the circular pattern
        circularFeat = circularFeats.add(circularFeatInput)

        return

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
