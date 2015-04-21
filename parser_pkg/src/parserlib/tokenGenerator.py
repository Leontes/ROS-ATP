#!/usr/bin/python

import re
from parserlib import symbols


class tokenGenerator(object):
	"""docstring for tokenGenerator"""

	def __init__(self, pddlFile):
		self.pddlFile = pddlFile
		self.line = ""
		self.rePattern = r"""\s*(,@|[('`,)]|"(?:[\\].|[^\\"])*"|;.*|[^\s('"`,;)]*)(.*)"""
		self.quotes = {"'":symbols._quote, "`":symbols._quasiquote, ",":symbols._unquote, ",@":symbols._unquotesplicing}


	def getNextToken(self):
		"Return the next token, reading new text into line buffer if needed."
		while True:
			if self.line == "": 
				self.line = self.pddlFile.readline()
			if self.line == "": 
				return symbols.eof_object

			token, self.line = re.match(self.rePattern, self.line).groups()
			if token != "" and not token.startswith(';;'):
				#print(token)
				return token

	def readToken(self, token):
		if '(' == token: 
			L = []
			while True:
				token = self.getNextToken()
				#print(token)
				if token == ')':
					#print(L)
					return L
				else:
					L.append(self.readToken(token))
		elif ')' == token:
			raise SyntaxError('unexpected )')
		elif token in self.quotes:
			return [self.quotes[token], self.readAll()]
		elif token is symbols.eof_object:
			raise SyntaxError('unexpected EOF in list')
		else:
			return self.atomic(token)


	def readAll(self):
		"Read a Scheme expression from an input port."
		token = self.getNextToken() 
		#print(token)
		if token == symbols.eof_object:
			return symbols.eof_object
		else:
			return self.readToken(token)



	def atomic(self, token):
		'Numbers become numbers; #t and #f are booleans; "..." string; otherwise Symbol.'
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
				return symbols.Sym(token)
