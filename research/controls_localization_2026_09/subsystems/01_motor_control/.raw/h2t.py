import sys,re,html
from html.parser import HTMLParser
class P(HTMLParser):
    def __init__(s): super().__init__(); s.o=[]; s.skip=0
    def handle_starttag(s,t,a):
        if t in('script','style','nav','footer','svg','noscript','header'): s.skip+=1
        if t in('p','div','br','li','tr','h1','h2','h3','h4','pre','table','section'): s.o.append('\n')
        if t in('h1','h2','h3','h4'): s.o.append('#'*int(t[1])+' ')
        if t=='li': s.o.append('- ')
        if t in('td','th'): s.o.append(' | ')
    def handle_endtag(s,t):
        if t in('script','style','nav','footer','svg','noscript','header'): s.skip=max(0,s.skip-1)
    def handle_data(s,d):
        if not s.skip: s.o.append(d)
p=P(); p.feed(open(sys.argv[1],encoding='utf-8',errors='ignore').read())
t=''.join(p.o); t=re.sub(r'[ \t]+',' ',t); t=re.sub(r'\n\s*\n+','\n\n',t)
open(sys.argv[2],'w').write(f"<!-- Source: {sys.argv[3]} (fetched 2026-09-27, HTML converted to text) -->\n\n"+t.strip()+"\n")
