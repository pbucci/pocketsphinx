#!/usr/bin/env python
import sys
import re

# Import dummy data
import intentmap
import objectlist
import identifiers
import determinants

# Dictionary generation and management
class DictionaryHandler:
	def __init__(self):
		self.intents = intentmap.intents
		self.objects = objectlist.objects
		self.identifiers = identifiers.identifiers
		self.determinants = determinants.determinants
		self.corpus = []

	def generateCorpus(self):
		for (intents,synonyms) in self.intents.iteritems():
			for s in synonyms:
				for o in self.objects:
					self.corpus.append(s + " " + o)

	def parseTest(self,string):
		return str.split(string)[2]

	def getVerb(self,string):
		s = str.split(str.lower(string))
		for w in s:
			print("Checking " + w)
			for i in verbs:
				if (i == w):
					return "Verb is " + w

	def checkIntents(self,string):
		strings = string.lower().split()
		strings.reverse()
		returnStrings = []
		for st in strings:
			for (intent,synonyms) in self.intents.iteritems():
				for s in synonyms:
					if (st == s):
						return intent," ".join(returnStrings)
			returnStrings.insert(0,st)

	def checkIdentifiers(self,string):
		strings = string.lower().split()
		for i in self.identifiers:
			if (strings[0] == i):
				strings.pop(0)
				return i," ".join(strings)

	def checkDeterminants(self,string):
		strings = string.lower().split()
		for d in self.determinants:
			if (strings[0] == d):
				strings.pop(0)
				return d, " ".join(strings)

	def checkObjects(self,string):
		strings = string.lower().split()
		for o in self.objects:
			if (strings[0] == o):
				strings.pop(0)
				return o, " ".join(strings)

	def checkList(self,string,list):
		strings = string.lower().split()
		for l in list:
			if (strings[0] == l):
				strings.pop(0)
				return l, " ".join(strings)


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

	def findFlex(self,string,regex):
		r = ".*"
		reg = r + regex + r
		return re.match(re.compile(reg),string)

# Main function
if __name__ == "__main__":
	s = DictionaryHandler()
	print(s.checkIntents("GO AND GET ME A SODA"))
	print(s.checkIntents("I WOULD LIKE YOU TO PLEASE GO AND GET ME A SODA"))
	print(s.checkIdentifiers("ME A SODA"))
	print(s.checkDeterminants("A SODA"))
	print(s.checkObjects("SODA POP"))
	print(s.checkIdentifiers("HALT"))
	print("\nFlex Tests:\n")
	print(s.checkList("ME A SODA",s.identifiers))
	print(s.checkList("A SODA",s.determinants))
	print(s.checkList("SODA POP",s.objects))


# Notes for phrase parsing
""" 
The structure of a sentence as we are attempting to understand it will be as such:

Ext			= Extraneous
Name		= A person's name
D			= Determinant
ID			= Identifier
V			= Verb
N			= Noun
P			= Preposition
NP			= Noun phrase = (D)(N)
PP			= Prepositional phrase = (P)(NP)
VP			= Verb phrase = (Ext)(V)


Sentence	= (VP)(ID?)(NP?)(PP?)

The justification is that all robot commands are essentially simple imperatives, so all words before a VP should be understood as extraneous.

Example 1:
	
	"GO GET ME A COKE"
	
	VP = GO GET
	ID = ME
	NP = A COKE
	
	->

	V  = GET
	ID = ME
	D  = A
	N  = COKE

Verbs are mapped to Intents, nouns are mapped to objects, identifiers are mapped to names, and prepositions are mapped to object metadata. In the above example, we can think of the mapping structure as such:

	GET ME A COKE -> intent(subject,object,metadata) -> retrieve(me,coke,null)
	
	
Example 2:

	"GO GET DAVID A COKE FROM THE FRIDGE"
	
	VP = GO GET
	ID = DAVID
	NP = A COKE
	PP = FROM THE FRIDGE

	->

	V  = GET
	ID = DAVID
	D  = A
	N  = COKE
	P  = FROM
	D  = THE
	N  = FRIDGE
	
	-> retrieve(david,coke,location:fridge)
	
Note: this is all conceptual at the moment, much thanks to David Marino for helping out with the linguistic analysis. Also, no sentence will include more than one prepositional phrase at the moment.
	
"""