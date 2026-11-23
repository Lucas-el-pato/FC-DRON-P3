import sys, numpy as np, pandas as pd
for f in sys.argv[1:]:
    d=pd.read_csv(f); d=d[d.throttle>=0.2].reset_index(drop=True)
    t=d.time_s.values; dt=np.median(np.diff(t))
    out=[]
    for ax in ['roll','pitch']:
        s=np.clip(d['stick_'+ax].values,-1,1); a=d['angle_'+ax].values; tgt=30*s
        ds=np.abs(np.gradient(s,dt))
        # transitions: stick changes by >0.3 within 120 ms, then steady (|ds|<0.5/s) for 0.5 s
        W=int(0.12/dt); H=int(0.5/dt); res=[]; i=0
        while i<len(s)-W-H-1:
            if abs(s[i+W]-s[i])>0.3 and np.all(abs(s[i+W:i+W+H]-s[i+W])<0.05):
                tg0=tgt[i]; tg1=np.mean(tgt[i+W:i+W+H]); amp=tg1-tg0; sg=np.sign(amp)
                seg=(a[i:i+W+H]-a[i])*sg; A=abs(amp)
                if A>6:
                    t50=np.argmax(seg>=0.5*A)*dt*1e3
                    t90=np.argmax(seg>=0.9*A)*dt*1e3 if np.any(seg>=0.9*A) else np.nan
                    ov=seg.max()/A
                    sse=np.mean((tg1-a[i+W+H//2:i+W+H])*sg)
                    res.append((A,t50,t90,ov,sse))
                i+=W+H; continue
            i+=1
        R=np.array(res)
        if len(R): out.append(f"{ax}: n={len(R)} amp {np.median(R[:,0]):.0f}deg t50 {np.nanmedian(R[:,1]):.0f}ms t90 {np.nanmedian(R[:,2]):.0f}ms pico/obj {np.median(R[:,3]):.2f} (p90 {np.percentile(R[:,3],90):.2f}) err final {np.median(R[:,4]):.1f}deg")
    print(f, *out, sep='\n  ')
