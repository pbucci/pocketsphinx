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

	# Checks a whole sentence for first value in a
	# list of values for each key in a dictionary,
	# returns a tuple of key and the rest of the string
	def checkDictionary(self,string,dictionary):
		strings = string.lower().split()
		strings.reverse()
		returnStrings = []
		for s in strings:
			for (key,values) in dictionary.iteritems():
				for v in values:
					if (s == v):
						return key," ".join(returnStrings)
			returnStrings.insert(0,s)
		return "none", " ".join(returnStrings)

	# Checks the first word of string against a list
	# Returns a tuple of the word and the rest of the
	# string if found, otherwise, the first element
	# of the tuple is "None"
	def checkList(self,string,list):
		strings = string.lower().split()
		for l in list:
			if (strings[0] == l):
				strings.pop(0)
				return l, " ".join(strings)
		return "none"," ".join(strings)

	# Calls word parsing functions to classify each word
	def parseSentence(self,string):
		intent	= self.checkDictionary(string,self.intents)
		id		= self.checkList(intent[1],self.identifiers)
		det		= self.checkDictionary(id[1],self.determinants)
		object	= self.checkList(det[1],self.objects)
		return intent[0],id[0],det[0],object[0]


	def makeMessage(self,string):
		s = self.parseSentence(string)
		i = "Intent		: " + s[0] + "\n"
		o = "Object		: " + s[3] + "\n"
		r = "Recipient	: " + s[1] + "\n"
		c = "Count		: " + s[2] + "\n"
		return i + o + r + c

# Main function
if __name__ == "__main__":
	s = DictionaryHandler()
	print(s.makeMessage("I WOULD LIKE YOU TO PLEASE GO AND GET ME A SODA"))
	print(s.makeMessage("CAN YOU GET DAVID A SODA PLEASE"))
	print(s.makeMessage("IS THERE ANY CHANCE THAT YOU CAN FIND PAUL SOME PEPSI"))


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