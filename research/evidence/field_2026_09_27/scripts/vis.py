import math, sys
from datetime import datetime, timezone, timedelta
from sgp4.api import Satrec, jday
LAT, LON, H = 34.0593, -117.8219, 230.0
def ecef_site():
    a,e2=6378137.0,6.69437999014e-3; la,lo=math.radians(LAT),math.radians(LON)
    N=a/math.sqrt(1-e2*math.sin(la)**2)
    return ((N+H)*math.cos(la)*math.cos(lo),(N+H)*math.cos(la)*math.sin(lo),(N*(1-e2)+H)*math.sin(la))
def gmst(jd):
    t=(jd-2451545.0)/36525; g=280.46061837+360.98564736629*(jd-2451545.0)+0.000387933*t*t
    return math.radians(g%360)
def elev(sat, dt):
    jd,fr=jday(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second)
    e,r,v=sat.sgp4(jd,fr)
    if e: return None
    th=gmst(jd+fr); x=r[0]*math.cos(th)+r[1]*math.sin(th); y=-r[0]*math.sin(th)+r[1]*math.cos(th); z=r[2]
    sx,sy,sz=ecef_site(); dx,dy,dz=x*1000-sx,y*1000-sy,z*1000-sz
    la,lo=math.radians(LAT),math.radians(LON)
    up=math.cos(la)*math.cos(lo)*dx+math.cos(la)*math.sin(lo)*dy+math.sin(la)*dz
    return math.degrees(math.asin(up/math.sqrt(dx*dx+dy*dy+dz*dz)))
def load(f):
    L=open(f).read().strip().splitlines(); return [Satrec.twoline2rv(L[i+1],L[i+2]) for i in range(0,len(L)-2,3)]
sys_={g:load(g+'.tle') for g in ('gps-ops','glo-ops','galileo','beidou')}
def count(dt, mask):
    return {g:sum(1 for s in sats if (el:=elev(s,dt)) is not None and el>=mask) for g,sats in sys_.items()}
now=datetime.now(timezone.utc)
fri=datetime(2026,9,26,0,29,tzinfo=timezone.utc)   # Friday 2026-09-25 17:29 PDT (first FIXED)
for label,dt in (('now',now),('Friday 17:29 PDT (FIXED)',fri)):
    for mask in (10,15):
        c=count(dt,mask); print(f"{label:28s} mask {mask}°: GPS {c['gps-ops']:2d}  GLONASS {c['glo-ops']:2d}  Galileo {c['galileo']:2d}  BeiDou {c['beidou']:2d}  | without BeiDou: {c['gps-ops']+c['glo-ops']+c['galileo']}")
