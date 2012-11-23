#!/usr/bin/env python
from os import environ, system

if __name__ == "__main__":
	if "COMMAND" in environ.keys():
		system(environ["COMMAND"])
