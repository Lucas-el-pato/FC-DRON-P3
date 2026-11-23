import sys, numpy as np, pandas as pd
LIM=30.0; KL=5.0; FFG=0.5; FC=15.0; SAT=150.0
def pt1(x, fc, dt):
    k=dt/(dt+1/(2*np.pi*fc)); y=np.empty_like(x); s=x[0]
    for i,v in enumerate(x): s+=k*(v-s); y[i]=s
    return y
def events(t, s, a, tgt):
    dt=np.median(np.diff(t)); n=len(s); out_step=[]; out_rel=[]
    i=0; w=int(0.15/dt)
    while i<n-int(1.0/dt):
        if abs(s[i])<0.1 and abs(s[i+w])>0.5:
            j=i+w; sg=np.sign(s[j])
            hold=s[j:j+int(0.4/dt)]
            if np.all(np.sign(hold)==sg) and np.all(abs(hold)>0.4):
                # start of the move
                k0=i+np.argmax(abs(s[i:j+1])>0.1)
                tg=np.median(tgt[j:j+int(0.4/dt)]); a0=a[k0]
                seg=a[k0:k0+int(0.6/dt)]
                d=(seg-a0)*sg; amp=(tg-a0)*sg
                if amp>8:
                    t50=np.argmax(d>=0.5*amp)*dt*1e3
                    t90=np.argmax(d>=0.9*amp)*dt*1e3 if np.any(d>=0.9*amp) else np.nan
                    out_step.append((t[k0],amp,t50,t90,d.max()/amp))
            i=j+int(0.4/dt); continue
        if abs(s[i])>0.5 and abs(s[i+w])<0.1:
            sg=np.sign(s[i]); k0=i+np.argmax(abs(s[i:i+w+1])<0.5)
            hold=s[i+w:i+w+int(0.6/dt)]
            if np.all(abs(hold)<0.1):
                a0=a[k0]*sg
                seg=a[k0:k0+int(0.8/dt)]*sg
                bounce=-seg.min()
                within=np.where(abs(seg)>2.0)[0]
                settle=(within[-1]+1)*dt*1e3 if len(within) else 0
                if a0>8: out_rel.append((t[k0],a0,bounce,settle))
            i=i+w+int(0.6/dt); continue
        i+=1
    return out_step,out_rel
for f in sys.argv[1:]:
    d=pd.read_csv(f); d=d[d.throttle>=0.2]
    t=d.time_s.values; dt=np.median(np.diff(t))
    print(f"== {f}  {len(d)*dt:.0f} s con throttle>=20%")
    for ax in ['roll','pitch']:
        s=np.clip(d['stick_'+ax].values,-1,1); a=d['angle_'+ax].values; sp=d['setpoint_'+ax].values
        tgt=s*LIM
        c=np.corrcoef(tgt,a)[0,1]
        # lag via xcorr on detrended
        x=tgt-tgt.mean(); y=a-a.mean(); best=(0,-1)
        for L in range(0,int(0.4/dt)):
            r=np.dot(x[:len(x)-L],y[L:])/len(x[L:])
            if r>best[1]: best=(L,r)
        err=tgt-a
        sm=pt1(pt1(tgt,FC,dt),FC,dt); ff=np.clip(FFG*np.gradient(sm,dt),-SAT,SAT)
        lvl=err*KL
        ffobs=sp-lvl
        moving=abs(ff)>5
        sat=np.mean(abs(sp)>=SAT-0.5)*100
        steps,rels=events(t,s,a,tgt)
        print(f"  {ax}: corr {c:.3f}  retardo angulo {best[0]*dt*1e3:.0f} ms  err rms {np.sqrt(np.mean(err**2)):.1f} deg (p99 {np.percentile(abs(err),99):.1f})  sp saturado {sat:.1f}%")
        print(f"     FF sim p99 {np.percentile(abs(ff),99):.0f} dps, sp-level observado p99 {np.percentile(abs(ffobs),99):.0f} dps, con stick quieto |ff| med {np.median(abs(ff[~moving])):.2f}")
        if steps:
            S=np.array(steps); print(f"     {len(S)} entradas: amp med {np.median(S[:,1]):.0f} deg  t50 {np.nanmedian(S[:,2]):.0f} ms  t90 {np.nanmedian(S[:,3]):.0f} ms  pico/obj {np.median(S[:,4]):.2f} (max {S[:,4].max():.2f})")
        if rels:
            R=np.array(rels); print(f"     {len(R)} sueltas: desde {np.median(R[:,1]):.0f} deg  rebote med {np.median(R[:,2]):.1f} deg (max {R[:,2].max():.1f})  asentado <2deg {np.median(R[:,3]):.0f} ms")
