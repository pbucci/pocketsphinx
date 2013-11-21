#!/usr/bin/env python
import sys
import re
import intentmap
import objectlist
import corpus

# Import dummy data
intents = intentmap.intents
objects = objectlist.objects
corpus = corpus.corpus

# VP finder
class SentenceParser:
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
		strings = str.split(str.lower(string))
		for st in strings:
			for (intent,synonyms) in intents.iteritems():
				for s in synonyms:
					if (st == s):
						return intent

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
	hd = HelloWorld()
	hd.main()
	r = RegexTests()
	s = SentenceParser();
	for w in strings:
		print(r.findCoke(w))
	for w in strings:
		for n in nouns:
			print(r.findFlex(w,n))
	print(s.parseTest("THIS IS A SENTENCE"))
	print(s.getVerb("PLEASE GO AND GET ME A COKE"))
	print(s.checkIntents("PLEASE GO AND GET ME A COKE"))
	print(s.checkIntents("PLEASE BRING ME A COKE"))
	print(s.checkIntents("RUN AROUND AND THEN HALT"))

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