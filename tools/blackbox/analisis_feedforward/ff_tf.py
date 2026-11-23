import sys, numpy as np, pandas as pd
from scipy import signal
fr=[0.5,1,2,3,4,6]
for f in sys.argv[1:]:
    d=pd.read_csv(f); d=d[d.throttle>=0.2]
    fs=1/np.median(np.diff(d.time_s.values)); line=[f]
    for ax in ['roll','pitch']:
        x=30*np.clip(d['stick_'+ax].values,-1,1); y=d['angle_'+ax].values
        fq,Pxy=signal.csd(x,y,fs=fs,nperseg=4096); _,Pxx=signal.welch(x,fs=fs,nperseg=4096); _,C=signal.coherence(x,y,fs=fs,nperseg=4096)
        H=Pxy/Pxx; s=[]
        for F in fr:
            k=np.argmin(abs(fq-F)); s.append(f"{F:g}Hz {abs(H[k]):.2f}/{np.degrees(np.angle(H[k])):.0f}°(c{C[k]:.2f})")
        m=(fq>0.3)&(fq<6)&(C>0.6)
        bw=fq[m][np.argmax(abs(H[m])<0.707)] if np.any(abs(H[m])<0.707) else np.nan
        line.append(f"{ax}: "+"  ".join(s)+f"  pico |H| {abs(H[m]).max():.2f} @ {fq[m][np.argmax(abs(H[m]))]:.1f}Hz")
    print(*line,sep='\n  ')
