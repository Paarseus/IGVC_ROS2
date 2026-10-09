import math, statistics as st
from datetime import datetime, timezone, timedelta
exec(open('vis.py').read().split("sys_={")[0])
sys_={g:load(g+'.tle') for g in ('gps-ops','glo-ops','galileo')}
rows=[]
for l in open('satlog.txt'):
    if '###' in l or 'sats=' not in l: continue
    t,rest=l.split(' ',1); s=int(rest.split('sats=')[1].split()[0])
    rows.append((t,s))
# local PDT -> UTC (+7h), date 2026-09-27
def utc(t):
    h,m,s=map(int,t.split(':')); return datetime(2026,9,27,h,m,s,tzinfo=timezone.utc)+timedelta(hours=7)
bins={}
for t,s in rows:
    k=t[:4]+('0' if int(t[4])<5 else '5')   # 5-minute bins
    bins.setdefault(k,[]).append((t,s))
print(f"{'bin':6s} {'robot used':>10s} {'GPS>10':>7s} {'GPS>5':>6s} {'GLO>10':>7s} {'GAL>10':>7s} {'all>10':>7s}")
xs=[];ys=[];zs=[]
for k,v in sorted(bins.items()):
    dt=utc(v[len(v)//2][0]); med=st.median(s for _,s in v)
    c10={g:sum(1 for s in sats if (el:=elev(s,dt)) is not None and el>=10) for g,sats in sys_.items()}
    g5=sum(1 for s in sys_['gps-ops'] if (el:=elev(s,dt)) is not None and el>=5)
    allc=sum(c10.values())
    print(f"{k:6s} {med:10.1f} {c10['gps-ops']:7d} {g5:6d} {c10['glo-ops']:7d} {c10['galileo']:7d} {allc:7d}")
