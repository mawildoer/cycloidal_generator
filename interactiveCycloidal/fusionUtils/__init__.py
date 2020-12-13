""" Utilities to handle the process of accepting inputs from Fusion.
    The process is rather complicated and confusing so much of it has
    been relocated here to prevent confusion """

import adsk.core
import adsk.fusion
import traceback
import math


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    """ Executes the reading of parameters and the building of the object """
    def __init__(self, app, ui, object_class, input_parameters):
        super().__init__()
        self.object_class = object_class
        self.app = app
        self.parameters = input_parameters
        self.ui = ui

    def notify(self, args):
        """ Builds the object from the given inputs """
        try:
            units_mgr = self.app.activeProduct.unitsManager
            command = args.firingEvent.sender
            inputs = command.commandInputs

            for current_input in inputs:
                test_parameter = self.parameters.parameter_dict[current_input.id]
                self.object_class.parameters[current_input.id] =\
                     units_mgr.evaluateExpression(current_input.expression, test_parameter.units)

            self.object_class.build(self.app, self.ui)
            args.isValidResult = True

        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class CommandDestroyHandler(adsk.core.CommandEventHandler):
    """ Terminates the script cleanly """
    def __init__(self, ui):
        super().__init__()
        self.ui = ui

    def notify(self, args):
        try:
            # when the command is done, terminate the script
            # this will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    """ Called by run to create the object. Calls the execute handler """
  
    def __init__(self, app, ui, object_class, input_parameters, handlers):
        super().__init__()  
        self.object_class = object_class
        self.app = app    
        self.parameters = input_parameters
        self.ui = ui
        self.handlers = handlers

    def notify(self, args):
        """ Runs other handlers to create the object """
        try:
            cmd = args.command
            cmd.isRepeatable = False
            onExecute = CommandExecuteHandler(self.app, self.ui,  self.object_class, self.parameters)
            cmd.execute.add(onExecute)
            onExecutePreview = CommandExecuteHandler(self.app, self.ui, self.object_class, self.parameters)
            cmd.executePreview.add(onExecutePreview)
            onDestroy = CommandDestroyHandler(self.ui)
            cmd.destroy.add(onDestroy)
            # keep the handler referenced beyond this function
            self.handlers.append(onExecute)
            self.handlers.append(onExecutePreview)
            self.handlers.append(onDestroy)

            #define the inputs
            inputs = cmd.commandInputs
            for parameter in self.parameters.parameter_list:
                init_value = adsk.core.ValueInput.createByReal(parameter.default_value)
                inputs.addValueInput(parameter.id, parameter.description, parameter.units, init_value)

        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class Parameter:
    """ A container for all parameters needed to create an input field """

    def __init__(self, name, units, description, default_value):
        self.id = name
        self.units = units
        self.description = description
        self.default_value = default_value


class Parameters:
    """ A container to hold parameters for input initialization """

    def __init__(self):
        """ Store a list of objects and a dictionary of object pointers
            for easy indexing by name and iterating """
        self.parameter_list = []
        self.parameter_dict = {}

    def addParameter(self, name, units, description, default_value):
        """ Add a parameter to the input box for the module.
            name: the varuable name that will hold the valie
            units: the units that the value will be converted to. "" for unitless
            description: the text which will appear with the box
            default_value: the initial value that will appear before being edited """

        new_param = Parameter(name, units, description, default_value)
        self.parameter_list.append(new_param)
        self.parameter_dict[name] = new_param


def createNewComponent(app):
    """ Create a new component in the active design """

    # Get the active design.
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    root_comp = design.rootComponent
    all_occs = root_comp.occurrences
    new_occ = all_occs.addNewComponent(adsk.core.Matrix3D.create())
    return new_occ.component


def run(parameters, default_name, createdObject):
    """ The default function run by Fusion """

    handlers = []
    app = adsk.core.Application.get()
    if app:
        ui = app.userInterface

    try:
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            ui.messageBox('It is not supported in current workspace, please change to MODEL workspace and try again.')
            return
        command_definitions = ui.commandDefinitions
        #check the command exists or not
        cmd_def = command_definitions.itemById(default_name)
        if not cmd_def:
            cmd_def = command_definitions.addButtonDefinition(default_name,
                    'Create ' + default_name,
                    'Create a' +  default_name,
                    '') # Edit last parameter to provide resources


        on_command_created = CommandCreatedHandler(app, ui, createdObject, parameters, handlers)
        cmd_def.commandCreated.add(on_command_created)
        # keep the handler referenced beyond this function
        handlers.append(on_command_created)
        inputs = adsk.core.NamedValues.create()
        cmd_def.execute(inputs)

        # prevent this module from being terminate when the script returns,
        # because we are waiting for event handlers to fire
        adsk.autoTerminate(False)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
