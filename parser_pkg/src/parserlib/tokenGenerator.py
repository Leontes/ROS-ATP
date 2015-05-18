#!/usr/bin/python

"""
tokenGenerator.py - Version 1.0 2015-05-15

Implements the lexic analisis for a pddl file

Copyright (c) 2015 Jose Angel Segura Muros.  All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""

import re

class Symbol(str): pass

def Sym(symbol, symbolTable={}):
    """Finds a simbol in a given simbol table. If doesn't exists the 
    function creates it

    Keyword arguments:
    symbol -- string object with the symbol name to find/create
    symbolTable -- stored symbol list
    """
    if symbol not in symbolTable:
    	symbolTable[symbol] = Symbol(symbol)
    return symbolTable[symbol]

symbolTable = {}

_quote = Sym("quote",symbolTable)
_quasiquote = Sym("quasiquote", symbolTable)
_unquote = Sym("unquote", symbolTable)
_unquotesplicing = Sym("unquote-splicing", symbolTable)

eof_object = Symbol('#<eof-object>')


class tokenGenerator(object):
	""" class tokenGenerator

	"""

	def __init__(self, pddlFile):
		""" Construct a object of the class tokenGenerator

		Keyword arguments:
		pddlFile -- file object 
		"""
		self.pddlFile = pddlFile
		self.line = ""
		self.rePattern = r"""\s*(,@|[('`,)]|"(?:[\\].|[^\\"])*"|;.*|[^\s('"`,;)]*)(.*)"""
		self.quotes = {"'":_quote, "`":_quasiquote, ",":_unquote, ",@":_unquotesplicing}


	def getNextToken(self):
		""" Return the next token

		"""
		while True:
			if self.line == "": 
				self.line = self.pddlFile.readline()
			if self.line == "": 
				return eof_object

			token, self.line = re.match(self.rePattern, self.line).groups()
			if token != "" and not token.startswith(';;'):
				return token


	def lexicAnalisis(self, token):
		""" Executes the lexic analisis recursively

		Keyword arguments:
		token -- symbol to check
		"""
		if '(' == token: 
			L = []
			while True:
				token = self.getNextToken()
				if token == ')':
					return L
				else:
					L.append(self.lexicAnalisis(token))
		elif ')' == token:
			raise SyntaxError('unexpected )')
		elif token in self.quotes:
			return [self.quotes[token], self.readAll()]
		elif token is eof_object:
			raise SyntaxError('unexpected EOF in list')
		else:
			return self.atomic(token)


	def readAll(self):
		""" Return a list of tokens

		"""
		token = self.getNextToken() 
		if token == eof_object:
			return eof_object
		else:
			return self.lexicAnalisis(token)



	def atomic(self, token):
		""" Converts a given token to a boolean/numer/float/symbol

		Keyword arguments:
		token -- token to be converted
		"""
		if token == '#t':
			return True
		elif token == '#f':
			return False
		elif token[0] == '"':
			return token[1:-1].decode('string_escape')
		try:
			return int(token)
		except ValueError:
			try:
				return float(token)
			except ValueError:
				return Sym(token)
