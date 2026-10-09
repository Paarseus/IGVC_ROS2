import math
from datetime import datetime, timezone
exec(open('vis.py').read().split("sys_={")[0])   # reuse load/elev helpers
def enu_vec(sat, dt):
    jd,fr=jday(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second)
    e,r,v=sat.sgp4(jd,fr)
    th=gmst(jd+fr); x=r[0]*math.cos(th)+r[1]*math.sin(th); y=-r[0]*math.sin(th)+r[1]*math.cos(th); z=r[2]
    sx,sy,sz=ecef_site(); dx,dy,dz=x*1000-sx,y*1000-sy,z*1000-sz
    la,lo=math.radians(LAT),math.radians(LON)
    E=-math.sin(lo)*dx+math.cos(lo)*dy; N=-math.sin(la)*math.cos(lo)*dx-math.sin(la)*math.sin(lo)*dy+math.cos(la)*dz
    U=math.cos(la)*math.cos(lo)*dx+math.cos(la)*math.sin(lo)*dy+math.sin(la)*dz
    d=math.sqrt(E*E+N*N+U*U); return (E/d,N/d,U/d)
def hdop(vecs, nsys):
    import itertools
    rows=[]
    for (e,n,u),k in vecs: rows.append([-e,-n,-u]+[1.0 if j==k else 0.0 for j in range(nsys)])
    m=len(rows[0]); A=[[sum(r[i]*r[j] for r in rows) for j in range(m)] for i in range(m)]
    # invert A (Gauss-Jordan)
    I=[[float(i==j) for j in range(m)] for i in range(m)]
    for c in range(m):
        p=max(range(c,m),key=lambda r:abs(A[r][c])); A[c],A[p]=A[p],A[c]; I[c],I[p]=I[p],I[c]
        f=A[c][c]; A[c]=[x/f for x in A[c]]; I[c]=[x/f for x in I[c]]
        for r in range(m):
            if r!=c:
                g=A[r][c]; A[r]=[a-g*b for a,b in zip(A[r],A[c])]; I[r]=[a-g*b for a,b in zip(I[r],I[c])]
    return math.sqrt(I[0][0]+I[1][1])
now=datetime.now(timezone.utc)
S={g:[s for s in load(g+'.tle') if (el:=elev(s,now)) is not None and el>=10] for g in ('gps-ops','glo-ops','galileo')}
sets={'GPS only':['gps-ops'],'GPS+GLONASS':['gps-ops','glo-ops'],'GPS+Galileo':['gps-ops','galileo'],'GPS+GLONASS+Galileo':['gps-ops','glo-ops','galileo']}
for name,gs in sets.items():
    vecs=[(enu_vec(s,now),k) for k,g in enumerate(gs) for s in S[g]]
    print(f"{name:22s}: {len(vecs):2d} satellites, HDOP {hdop(vecs,len(gs)):.2f}")
