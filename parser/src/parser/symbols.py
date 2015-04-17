

class Symbol(str): pass

def Sym(s, symbol_table={}):
    "Find or create unique Symbol entry for str s in symbol table."
    if s not in symbol_table:
    	symbol_table[s] = Symbol(s)
    return symbol_table[s]

symbols = {}

_quote = Sym("quote",symbols)
_if = Sym("quote", symbols)
_quasiquote = Sym("quasiquote", symbols)
_unquote = Sym("unquote", symbols)
_unquotesplicing = Sym("unquote-splicing", symbols)