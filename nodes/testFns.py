#!/usr/bin/env python
import sys
import re

# Constants
strings = ["get me a coke",
		   "go get a coke",
		   "i want you to get me a coke",
		   "find my coke",
		   "find my pepsi"]
verbs = ["get", "find"]
nouns = ["coke","pepsi"]
regex = ".*coke.*"


# Hello world for testing
class HelloWorld:
	def world(self):
		sys.stdout.write("Hello, world!\n")
		sys.stdout.flush()

	def main(self):
		self.world()

# Regex tests
class RegexTests:
	def findCoke(self,string):
		return re.match(re.compile(regex), string)

if __name__ == "__main__":
	hd = HelloWorld()
	hd.main()
	r = RegexTests()
	for w in strings:
		print(r.findCoke(w))
