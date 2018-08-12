import argparse
from importlib import import_module

from demosys.conf import settings
from demosys.utils.module_loading import import_string
from demosys import context, project, timeline


class CommandError(Exception):
    pass


class BaseCommand:
    help = ''

    def __init__(self):
        """should take cmd config"""
        pass

    def add_arguments(self, parser):
        """
        This method is for adding arguments to a command.
        When extending this class we define the arguments
        by adding it to the parser passed in.

        :param parser: The parser to add arguments to (standard argparse)
        """
        pass

    def handle(self, *args, **options):
        """
        The actual run logic for the command.

        :param args: arguments from the argparser
        :param options: keyword arguments from the argparser
        """
        raise NotImplementedError()

    def run_from_argv(self, argv):
        """
        Called by the system when executing the command from the command line.
        This should not be overridden.

        :param argv: Arguments from command line
        """
        parser = self.create_parser(argv[0], argv[1])
        options = parser.parse_args(argv[2:])
        cmd_options = vars(options)
        args = cmd_options.pop('args', ())
        self.handle(*args, **cmd_options)

    def print_help(self, prog_name, subcommand):
        """
        Prints the help text generated by the argument parser defined for this command.
        This method should not be overridden.

        :param prog_name: name of the program that started the command.
        :param subcommand: The subcommand name
        """
        parser = self.create_parser(prog_name, subcommand)
        parser.print_help()

    def create_parser(self, prog_name, subcommand):
        """
        Create argument parser and deal with ``add_arguments``.
        This method should not be overriden.

        :param prog_name: Name of the command (argv[0])
        :return: ArgumentParser
        """
        parser = argparse.ArgumentParser(prog_name, subcommand)
        # Add generic arguments here
        self.add_arguments(parser)
        return parser


class CreateCommand(BaseCommand):
    """Used for createproject and createeffect"""

    def validate_name(self, name):
        """
        Can the name be used as a python module or package?
        Raises ``ValueError`` if the name is invalid.

        :param name: the name to check
        """
        if not name:
            raise ValueError("Name cannot be empty")

        # Can the name be used as an identifier in python (module or package name)
        if not name.isidentifier():
            raise ValueError("{} is not a valid identifier".format(name))

    def try_import(self, name):
        """
        Attempt to import the name.
        Raises ``ImportError`` if the name cannot be imported.

        :param name: the name to import
        """
        try:
            import_module(name)
        except ImportError:
            pass
        else:
            raise ImportError("{} conflicts with an existing python module".format(name))


class RunCommand(BaseCommand):

    def create_window(self):
        return context.create_window()

    def create_project(self, *args, **kwargs):
        project.instance = import_string(settings.PROJECT)(*args, **kwargs)
        return project.instance

    def create_timeline(self, project):
        timeline.instance = import_string(settings.TIMELINE)(project)
        return timeline.instance
