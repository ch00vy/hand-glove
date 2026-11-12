"""Rokoko Smartgloves -> MyoHand local process launcher (Windows, Tkinter)."""
import json
try:
    import keyring  # Optional: stores Wi-Fi passwords in Windows Credential Manager.
except ImportError:
    keyring = None
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox

from config import SDK_HOME

SDK = SDK_HOME
PROJECT = Path(__file__).resolve().parent
SETTINGS = PROJECT / '.rokoko_myohand_gui.json'
C = {'bg':'#111820','panel':'#1b2531','input':'#151e28','border':'#344354',
     'fg':'#edf3fa','muted':'#9aacc1','blue':'#3377e8','green':'#4ad19b',
     'red':'#ff7883','amber':'#f5bd63'}

class App:
    def __init__(self, root):
        self.root = root
        root.title('Rokoko Smartgloves | MyoHand Control Center')
        root.geometry('1280x850')
        root.minsize(1050, 650)
        root.configure(bg=C['bg'])
        self.q = queue.Queue()
        self.processes = {}
        self.busy = set()
        self.pipeline_jobs = []
        self.closing = False
        self.password = tk.StringVar()
        self.ssid = tk.StringVar()
        self.receiver_port = tk.StringVar(value='14041')
        self.driver_port = tk.StringVar(value='12276')
        self.solver_port = tk.StringVar(value='12277')
        self.wifi_status = tk.StringVar(value='Belum dikonfigurasi')
        self.stage_status = {k:tk.StringVar(value='Stopped') for k in ['driver','solver','viewer']}
        self.sdk = tk.StringVar(value=str(SDK))
        self.project = tk.StringVar(value=str(PROJECT))
        self.show_pw = tk.BooleanVar(value=False)
        self.load_settings()
        if keyring is not None and self.ssid.get():
            try:
                saved = keyring.get_password('rokoko-myohand-gui', self.ssid.get())
                if saved: self.password.set(saved)
            except Exception: pass
        self.build()
        self.root.after(130, self.consume)
        self.root.after(900, self.poll_processes)
        root.protocol('WM_DELETE_WINDOW', self.close)

    def load_settings(self):
        try:
            s=json.loads(SETTINGS.read_text(encoding='utf-8'))
            for name,var in [('ssid',self.ssid),('receiver_port',self.receiver_port),
                             ('driver_port',self.driver_port),('solver_port',self.solver_port),
                             ('sdk',self.sdk),('project',self.project)]:
                if isinstance(s, dict) and name in s: var.set(str(s[name]))
        except (FileNotFoundError, ValueError, OSError): pass

    def save_settings(self):
        # Password deliberately never saved to plaintext JSON.
        s={k:v.get() for k,v in [('ssid',self.ssid),('receiver_port',self.receiver_port),
                 ('driver_port',self.driver_port),('solver_port',self.solver_port),
                 ('sdk',self.sdk),('project',self.project)]}
        try:
            for port in ('receiver_port','driver_port','solver_port'):
                self.validate_port(s[port])
            SETTINGS.write_text(json.dumps(s,indent=2),encoding='utf-8')
            if self.password.get() and keyring is not None:
                keyring.set_password('rokoko-myohand-gui',self.ssid.get(),self.password.get())
                pw_info = 'Password disimpan melalui Windows Credential Manager.'
            elif self.password.get():
                pw_info = 'Password hanya tersimpan selama aplikasi terbuka. Install keyring untuk penyimpanan aman.'
            else:
                pw_info = 'Tidak ada password untuk disimpan.'
            self.log('Settings',f'Konfigurasi disimpan: {SETTINGS}. Password tidak disimpan dalam JSON.')
            messagebox.showinfo('Save', 'Konfigurasi disimpan. '+pw_info)
        except Exception as e:
            messagebox.showerror('Save gagal',str(e))

    def label(self,parent,text,muted=False,**kwargs):
        return tk.Label(parent,text=text,bg=parent['bg'],fg=C['muted'] if muted else C['fg'],
                        font=('Segoe UI',10),**kwargs)

    def panel(self,parent):
        f=tk.Frame(parent,bg=C['panel'],highlightbackground=C['border'],highlightthickness=1,padx=17,pady=14)
        return f

    def entry(self,parent,var,show=None,width=None):
        e=tk.Entry(parent,textvariable=var,bg=C['input'],fg=C['fg'],insertbackground=C['fg'],
                   relief='flat',highlightbackground=C['border'],highlightcolor=C['blue'],
                   highlightthickness=1,font=('Segoe UI',10),show=show or '',width=width)
        return e

    def button(self,parent,title,callback,accent=False):
        return tk.Button(parent,text=title,command=callback,bg=C['blue'] if accent else '#304052',
                         fg='white',activebackground='#4787ee' if accent else '#43556b',
                         activeforeground='white',relief='flat',padx=14,pady=7,
                         cursor='hand2',font=('Segoe UI Semibold',10))

    def build(self):
        hdr=tk.Frame(self.root,bg=C['bg']);hdr.pack(fill='x',padx=20,pady=(15,12))
        tk.Label(hdr,text='Rokoko  /  MyoHand',font=('Segoe UI Semibold',20),
                 bg=C['bg'],fg=C['fg']).pack(anchor='w')
        self.label(hdr,'Smartgloves network setup • Device SDK • Hand Solver • Real-time MuJoCo',True).pack(anchor='w')
        paths=self.panel(self.root);paths.pack(fill='x',padx=20,pady=(0,12))
        for i,(name,var) in enumerate((('SDK folder',self.sdk),('Project folder',self.project))):
            self.label(paths,name,True).grid(row=i,column=0,sticky='w',padx=(0,10),pady=3)
            self.entry(paths,var).grid(row=i,column=1,sticky='ew',pady=3)
        paths.grid_columnconfigure(1,weight=1)
        content=tk.Frame(self.root,bg=C['bg']);content.pack(fill='both',expand=True,padx=20)
        leftshell=tk.Frame(content,bg=C['bg']);leftshell.pack(side='left',fill='both',expand=True,padx=(0,12))
        self.left_canvas=tk.Canvas(leftshell,bg=C['bg'],highlightthickness=0,width=650)
        scroll=tk.Scrollbar(leftshell,command=self.left_canvas.yview)
        scroll.pack(side='right',fill='y')
        self.left_canvas.configure(yscrollcommand=scroll.set)
        self.left_canvas.pack(side='left',fill='both',expand=True)
        left=tk.Frame(self.left_canvas,bg=C['bg'])
        window=self.left_canvas.create_window((0,0),window=left,anchor='nw')
        left.bind('<Configure>',lambda event:self.left_canvas.configure(scrollregion=self.left_canvas.bbox('all')))
        self.left_canvas.bind('<Configure>',lambda event:self.left_canvas.itemconfigure(window,width=event.width))
        def scroll_left(event):
            widget=event.widget
            while widget is not None:
                if widget is self.left_canvas:
                    self.left_canvas.yview_scroll(-int(event.delta/120),'units')
                    return 'break'
                widget=getattr(widget,'master',None)
        self.root.bind('<MouseWheel>',scroll_left)
        right=tk.Frame(content,bg=C['bg']);right.pack(side='left',fill='both',expand=True)
        wifi=self.panel(left);wifi.pack(fill='x',pady=(0,12))
        tk.Label(wifi,text='01   Wi-Fi Configuration',bg=C['panel'],fg=C['fg'],font=('Segoe UI Semibold',13)).pack(anchor='w')
        self.label(wifi,'Network configuration for Smartgloves',True).pack(anchor='w',pady=(0,10))
        g=tk.Frame(wifi,bg=C['panel']);g.pack(fill='x')
        self.label(g,'SSID',True).grid(row=0,column=0,sticky='w',pady=(0,5))
        self.label(g,'Password',True).grid(row=0,column=1,sticky='w',padx=(10,0),pady=(0,5))
        self.entry(g,self.ssid).grid(row=1,column=0,sticky='ew')
        self.pw_entry=self.entry(g,self.password,show='•');self.pw_entry.grid(row=1,column=1,sticky='ew',padx=(10,0))
        g.grid_columnconfigure(0,weight=1);g.grid_columnconfigure(1,weight=1)
        pwrow=tk.Frame(wifi,bg=C['panel']);pwrow.pack(fill='x',pady=(8,8))
        tk.Checkbutton(pwrow,text='Show password',variable=self.show_pw,command=self.toggle_pw,
                       bg=C['panel'],fg=C['muted'],activebackground=C['panel'],
                       activeforeground=C['fg'],selectcolor=C['input']).pack(side='left')
        self.label(pwrow,'Receiver port',True).pack(side='left',padx=(15,6))
        self.entry(pwrow,self.receiver_port,width=8).pack(side='left')
        br=tk.Frame(wifi,bg=C['panel']);br.pack(fill='x',pady=(4,8))
        self.button(br,'Save',self.save_settings).pack(side='left',padx=(0,8))
        self.button(br,'Set Wi-Fi',lambda:self.run_wifi('set'),True).pack(side='left',padx=(0,8))
        self.button(br,'Verify (Get)',lambda:self.run_wifi('get')).pack(side='left')
        statusrow=tk.Frame(wifi,bg=C['panel']);statusrow.pack(fill='x')
        self.label(statusrow,'Status: ',True).pack(side='left')
        tk.Label(statusrow,textvariable=self.wifi_status,bg=C['panel'],fg=C['amber'],font=('Segoe UI',10)).pack(side='left')
        self.label(wifi,'Jalankan SDK Driver dan sambungkan gloves via USB sebelum Set/Verify. Set Wi-Fi belum membuktikan koneksi ke access point.',True,
                   wraplength=450,justify='left').pack(anchor='w',pady=(9,0))
        ports=self.panel(left);ports.pack(fill='x',pady=(0,12))
        tk.Label(ports,text='Network / Stream Ports',bg=C['panel'],fg=C['fg'],font=('Segoe UI Semibold',12)).pack(anchor='w',pady=(0,8))
        r=tk.Frame(ports,bg=C['panel']);r.pack(fill='x')
        for col,(label,var) in enumerate((('Driver RGMP',self.driver_port),('Solver output',self.solver_port))):
            cell=tk.Frame(r,bg=C['panel']);cell.grid(row=0,column=col,sticky='ew',padx=(0,12))
            self.label(cell,label,True).pack(anchor='w');self.entry(cell,var).pack(fill='x',pady=5)
            r.grid_columnconfigure(col,weight=1)
        runner=self.panel(left);runner.pack(fill='x')
        tk.Label(runner,text='Pipeline Controller',bg=C['panel'],fg=C['fg'],font=('Segoe UI Semibold',13)).pack(anchor='w',pady=(0,6))
        for idx,(key,name,desc) in enumerate((('driver','02  SDK Driver','Raw sensors / RGMP TCP'),
                ('solver','03  Hand Solver','Joint pose / RGMP TCP / emit both'),
                ('viewer','04  MyoHand Retargeting','MuJoCo viewer · real_time/run_retargeting.py'))):
            line=tk.Frame(runner,bg=C['panel']);line.pack(fill='x',pady=6)
            leftpart=tk.Frame(line,bg=C['panel']);leftpart.pack(side='left',fill='x',expand=True)
            tk.Label(leftpart,text=name,bg=C['panel'],fg=C['fg'],font=('Segoe UI Semibold',10)).pack(anchor='w')
            self.label(leftpart,desc,True).pack(anchor='w')
            btns=tk.Frame(line,bg=C['panel']);btns.pack(side='right')
            self.button(btns,'Start',lambda k=key:self.start_stage(k),True).pack(side='left',padx=(0,5))
            self.button(btns,'Stop',lambda k=key:self.stop_stage(k)).pack(side='left')
            tk.Label(line,textvariable=self.stage_status[key],bg=C['panel'],fg=C['green'],width=10,
                     font=('Segoe UI',9)).pack(side='right',padx=6)
        row=tk.Frame(runner,bg=C['panel']);row.pack(fill='x',pady=(10,0))
        self.button(row,'Start Pipeline',self.start_all,True).pack(side='left',padx=(0,7))
        self.button(row,'Stop All',self.stop_all).pack(side='left')
        self.label(runner,'Jika proses SDK/Solver sudah berjalan di luar GUI, tidak dijalankan ulang.',True,
                   wraplength=460,justify='left').pack(anchor='w',pady=(10,0))
        logpanel=self.panel(right);logpanel.pack(fill='both',expand=True)
        head=tk.Frame(logpanel,bg=C['panel']);head.pack(fill='x',pady=(0,10))
        tk.Label(head,text='Live Console Logs',bg=C['panel'],fg=C['fg'],font=('Segoe UI Semibold',13)).pack(side='left')
        self.button(head,'Clear',self.clear_log).pack(side='right')
        f=tk.Frame(logpanel,bg=C['panel']);f.pack(fill='both',expand=True)
        self.console=tk.Text(f,bg='#0c121a',fg='#c9d8e8',insertbackground='white',wrap='word',
                             font=('Consolas',9),width=45,relief='flat',state='disabled',padx=10,pady=9)
        sb=tk.Scrollbar(f,command=self.console.yview);sb.pack(side='right',fill='y')
        self.console.configure(yscrollcommand=sb.set);self.console.pack(fill='both',expand=True)
        self.log('System','Ready. Start SDK Driver, connect gloves via USB, then Set/Verify Wi-Fi and start Solver/Viewer.')
        self.log('System','Password masked; saved securely via keyring if installed, never in JSON.')

    def toggle_pw(self): self.pw_entry.configure(show='' if self.show_pw.get() else '•')
    def log(self,source,msg): self.q.put(('log',source,str(msg)))
    def clear_log(self):
        self.console.configure(state='normal');self.console.delete('1.0','end');self.console.configure(state='disabled')
    @staticmethod
    def validate_port(value):
        p=int(value)
        if not 1<=p<=65535: raise ValueError('Port harus 1–65535')
        return p
    def script(self): return Path(self.sdk.get())/'examples/python/network_configuration.py'
    def python_cmd(self): return [sys.executable, '-B', '-u']

    def run_wifi(self,mode):
        if 'wifi' in self.busy:
            messagebox.showwarning('Wi-Fi','Proses konfigurasi Wi-Fi masih berjalan.');return
        script=self.script()
        if not script.is_file():
            messagebox.showerror('Script tidak ditemukan',str(script));return
        if mode=='set' and not self.ssid.get().strip():
            messagebox.showerror('SSID kosong','Isi SSID terlebih dahulu.');return
        try: p=self.validate_port(self.receiver_port.get())
        except ValueError as e: messagebox.showerror('Port tidak valid',str(e));return
        # Capture Tk values on the GUI thread. Workers never access Tk variables.
        password = self.password.get() if mode == 'set' else ''
        if mode == 'set' and (not password or any(c in password for c in '\r\n')):
            messagebox.showerror('Password tidak valid','Isi password WPA2 tanpa karakter baris baru.');return
        runner = Path(__file__).with_name('sdk_network_runner.py')
        cmd=self.python_cmd()+[str(runner),str(script),mode]
        if mode=='set':
            cmd+=['--ssid',self.ssid.get().strip(),'--receiver-address','auto',
                  '--receiver-port',str(p),'--security','WPA2']
        self.wifi_status.set('Running '+mode+'...');self.busy.add('wifi')
        self.log('Wi-Fi',f'Running {mode} (credentials hidden).')
        def worker():
            try:
                flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
                completed=subprocess.run(cmd,input=password+'\n' if mode=='set' else '',
                                         stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                         text=True,encoding='utf-8',errors='replace',
                                         env={**os.environ, 'PYTHONIOENCODING':'utf-8'},
                                         timeout=90,creationflags=flags)
                out=completed.stdout or ''
                if password: out=out.replace(password,'[REDACTED]')
                self.q.put(('wifi_done',completed.returncode,mode,out))
            except subprocess.TimeoutExpired:
                self.q.put(('wifi_done',-1,mode,'SDK timeout setelah 90 detik.'))
            except Exception as e:
                out=str(e)
                if password: out=out.replace(password,'[REDACTED]')
                self.q.put(('wifi_done',-1,mode,out))
        threading.Thread(target=worker,daemon=True).start()

    def cmds(self,key):
        sdk=Path(self.sdk.get());proj=Path(self.project.get())
        driver=str(sdk/'bin/rokoko-sdk.exe');solver=str(sdk/'bin/rkk-hand-solver.exe')
        python=str(proj/'.venv/Scripts/python.exe');script=str(proj/'real_time/run_retargeting.py')
        if key=='driver':
            return [driver,'-vv','--auto-stream-usb','--pipeline-formatter','rgmp',
                    '--rgmp-output-addr',f'127.0.0.1:{self.validate_port(self.driver_port.get())}',
                    '--driver-udp-port',str(self.validate_port(self.receiver_port.get()))],str(sdk),driver
        if key=='solver':
            return [solver,'--no-driver','--rgmp-addr',f'127.0.0.1:{self.validate_port(self.driver_port.get())}',
                    '--listen',f'127.0.0.1:{self.validate_port(self.solver_port.get())}','--emit','both'],str(sdk),solver
        return [python,'-u',script,'--host','127.0.0.1','--port',str(self.validate_port(self.solver_port.get()))],str(proj),script

    def external_running(self,exe_name):
        """Best-effort name-based check. Do not mistake this for guaranteed port ownership."""
        if os.name!='nt':return False
        try:
            flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
            result=subprocess.run(['tasklist','/FI',f'IMAGENAME eq {exe_name}','/FO','CSV','/NH'],
                 capture_output=True,text=True,timeout=5,creationflags=flags)
            return any(line.strip().strip('"').split('","')[0].lower()==exe_name.lower()
                       for line in result.stdout.splitlines())
        except Exception:return False

    def start_stage(self,key):
        if self.closing:return
        if key in self.processes and self.processes[key].poll() is None:
            self.log(key,'Already running; skip.');return
        if key in self.busy:return
        try: cmd,cwd,needed=self.cmds(key)
        except ValueError as e:messagebox.showerror('Invalid port',str(e));return
        if not Path(needed).is_file():
            messagebox.showerror('File not found',needed);return
        if key in ('driver','solver') and self.external_running(Path(cmd[0]).name):
            self.stage_status[key].set('External')
            self.log(key,'An external process with this executable name is running. Skipping to avoid duplication.')
            return
        if key == 'viewer' and not Path(cmd[0]).is_file():
            messagebox.showerror('Python tidak ditemukan',cmd[0]);return
        flags=getattr(subprocess,'CREATE_NEW_PROCESS_GROUP',0)
        if os.name=='nt':flags |= getattr(subprocess,'CREATE_NO_WINDOW',0)
        try:
            p=subprocess.Popen(cmd,cwd=cwd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                               text=True,encoding='utf-8',errors='replace',bufsize=1,creationflags=flags,
                               env={**os.environ, 'PYTHONIOENCODING':'utf-8',
                                    'ROKOKO_SDK_HOME':str(Path(self.sdk.get()).resolve())})
        except (OSError,ValueError) as e:
            messagebox.showerror(f'Failed to launch {key}',str(e));return
        self.processes[key]=p
        self.stage_status[key].set('Running')
        self.log(key,'Started PID '+str(p.pid)+' | '+ ' '.join(cmd))
        threading.Thread(target=self.read_output,args=(key,p),daemon=True).start()

    def read_output(self,key,p):
        try:
            for line in iter(p.stdout.readline,''):
                if line:self.log(key,line.rstrip())
        except Exception as e:self.log(key,'Log read error: '+str(e))
        finally:
            try:p.stdout.close()
            except Exception:pass

    def start_all(self):
        # Stagger launching to give upstream processes time to bind. Not proof of readiness.
        self.cancel_pipeline_jobs()
        self.start_stage('driver')
        self.pipeline_jobs = [
            self.root.after(1500,lambda:self.start_stage('solver')),
            self.root.after(3000,lambda:self.start_stage('viewer'))]

    def cancel_pipeline_jobs(self):
        for job in self.pipeline_jobs:
            self.root.after_cancel(job)
        self.pipeline_jobs.clear()

    def stop_stage(self,key):
        p=self.processes.get(key)
        if not p or p.poll() is not None:
            self.log(key,'No GUI-owned process to stop. External processes will not be killed.');return
        try:
            if os.name=='nt':
                flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
                subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],capture_output=True,
                               timeout=10,creationflags=flags)
            else:p.terminate()
            self.stage_status[key].set('Stopped')
            self.log(key,'Stop requested for GUI-owned process.')
        except Exception as e:self.log(key,'Stop error: '+str(e))

    def stop_all(self):
        self.cancel_pipeline_jobs()
        for key in ('viewer','solver','driver'): self.stop_stage(key)

    def poll_processes(self):
        for key,p in list(self.processes.items()):
            status=p.poll()
            if status is not None:
                self.stage_status[key].set('Stopped' if status==0 else f'Exit {status}')
                self.log(key,f'Process exited with code {status}.')
                del self.processes[key]
        self.root.after(900,self.poll_processes)

    def consume(self):
        for _ in range(80):
            try:item=self.q.get_nowait()
            except queue.Empty:break
            if item[0]=='log':
                _,source,msg=item
                self.console.configure(state='normal')
                self.console.insert('end',f'[{source}] {msg}\n')
                if int(self.console.index('end-1c').split('.')[0])>1800:
                    self.console.delete('1.0','400.0')
                self.console.see('end');self.console.configure(state='disabled')
            elif item[0]=='wifi_done':
                _,code,mode,out=item;self.busy.discard('wifi')
                self.wifi_status.set('Success' if code==0 else 'Failed')
                self.log('Wi-Fi',out.strip() or '(no output)')
                self.log('Wi-Fi',f'{mode} exited with code {code}.')
                if code!=0:
                    messagebox.showwarning('Wi-Fi command failed',
                        'Periksa log. Pastikan SDK Driver berjalan dan Smartgloves terhubung via USB.')
        self.root.after(130,self.consume)

    def close(self):
        if 'wifi' in self.busy:
            messagebox.showinfo('Wi-Fi masih berjalan','Tunggu konfigurasi Wi-Fi selesai sebelum menutup aplikasi.');return
        live=[k for k,p in self.processes.items() if p.poll() is None]
        if live:
            if not messagebox.askyesno('Exit','Stop GUI-owned processes ('+', '.join(live)+') and exit?'):
                return
            self.stop_all()
        self.closing = True
        self.cancel_pipeline_jobs()
        self.root.destroy()

def main():
    if os.name!='nt':
        print('This launcher is designed to execute the supplied Windows SDK paths.',file=sys.stderr)
    r=tk.Tk();App(r);r.mainloop()


if __name__ == '__main__':
    main()
