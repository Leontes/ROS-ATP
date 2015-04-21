#!/usr/bin/python

class Symbol(str): pass

def Sym(s, symbol_table={}):
    "Find or create unique Symbol entry for str s in symbol table."
    if s not in symbol_table:
    	symbol_table[s] = Symbol(s)
    return symbol_table[s]

symbolTable = {}

_quote = Sym("quote",symbolTable)
_if = Sym("quote", symbolTable)
_quasiquote = Sym("quasiquote", symbolTable)
_unquote = Sym("unquote", symbolTable)
_unquotesplicing = Sym("unquote-splicing", symbolTable)

eof_object = Symbol('#<eof-object>')