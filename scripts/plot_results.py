# Copyright (c) 2026 Zhouchun Huang.
# SPDX-License-Identifier: MIT
"""Redraw three numerical figures from archived raw files (matplotlib/numpy).

Outputs: generated_figure.png in each corresponding experiment directory.
Layouts are newly drawn using the historical plotting conventions.
"""
import argparse
import re
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
R=ROOT/'results'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,help='Save all three PNGs in this directory instead of their experiment directories')
args=parser.parse_args()
OUT=args.output_dir.resolve() if args.output_dir else None


def read_kpi(path):
    values={}
    for line in path.read_text().splitlines():
        match=re.match(r'^([^:]+):\s*([-+\d.eE]+)\s*$',line)
        if match: values[match[1].strip()]=float(match[2])
    return values


data={'copy_generation':[[float(v) for v in line.split()]
      for line in (R/'copy_generation/raw/copy_gen.out').read_text().splitlines()
      if line.strip() and not line.startswith('#')], 'max_copies':[], 'simultaneous':[]}
for m in [1,2,5,10,20,100,10000]:
    k=read_kpi(R/f'sensitivity/max_copies/raw/{m}/CCG-CSC/kpi.out')
    data['max_copies'].append([m,k['Total']/1000,k['Run time(sec)']])
for i in range(1,31):
    costs=[read_kpi(R/f'sensitivity/simultaneous/raw/{s}/{i}/CCG-CSC/kpi.out')['Total']
           for s in ['original','no_conic','no_multiple']]
    data['simultaneous'].append([i]+[c/1000 for c in costs]+[100*(c-costs[0])/costs[0] for c in costs[1:]])
figure_dirs={'copy_generation':R/'copy_generation',
             'sensitivity_max_copies':R/'sensitivity/max_copies',
             'sensitivity_simultaneous':R/'sensitivity/simultaneous'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False})


def save(fig,name):
    path=OUT/(name+'.png') if OUT else figure_dirs[name]/'generated_figure.png'
    path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,dpi=180,facecolor='white')
    print('Saved',path)
    plt.close(fig)


d=np.asarray(data['copy_generation'])
fig,ax=plt.subplots(figsize=(14,6),layout='constrained')
right=ax.twinx();x=np.arange(len(d));width=.36
labels=['Individual regular','Multiple regular','Individual CSC','Multiple CSC']
hatches=['','///','xx','...']
for offset,cols,color in [(-width/2,range(2,6),'white'),(width/2,range(7,11),'0.72')]:
    bottom=np.zeros(len(d))
    for col,hatch in zip(cols,hatches):
        ax.bar(x+offset,d[:,col],width,bottom=bottom,color=color,edgecolor='0.2',linewidth=.5,hatch=hatch)
        bottom+=d[:,col]
right.plot(x,d[:,11]/1e6,'o-',color='black',markersize=3,linewidth=1.2)
ax.set(xlabel='Iteration',ylabel='Number of copies',xticks=x,xticklabels=d[:,0].astype(int),ylim=(0,d[:,1].max()*1.18))
right.set(ylabel='Objective (million dollars)',ylim=(0,d[:,11].max()/1e6*1.18))
ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
handles=[Patch(facecolor='white',edgecolor='0.2',hatch=h,label=l) for l,h in zip(labels,hatches)]
handles += [Patch(facecolor=c,edgecolor='0.2',label=l) for c,l in [('white','Generated'),('0.72','Retained')]]
handles += [Line2D([],[],color='black',marker='o',markersize=3,label='Objective')]
ax.legend(handles=handles,ncol=2,loc='upper right',fontsize=9)
save(fig,'copy_generation')

d=np.asarray(data['max_copies']);x=np.arange(len(d))
fig,ax=plt.subplots(figsize=(7,4.8),layout='constrained');right=ax.twinx()
ax.bar(x,d[:,1],width=.55,color='0.85',edgecolor='black',label='Recovery cost')
right.plot(x,d[:,2],'o--',color='black',label='Runtime')
ax.set(xlabel='Maximum copies per iteration (M)',ylabel='Recovery cost (thousand dollars)',xticks=x,xticklabels=['1','2','5','10','20','100',r'$\infty$'])
ax.set_ylim(1300,1340);right.set(ylabel='Runtime (seconds)',ylim=(0,13))
ax.legend(loc='upper left');right.legend(loc='upper right')
save(fig,'sensitivity_max_copies')

d=np.asarray(data['simultaneous']);x=np.arange(len(d))
fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
for offset,col,label,hatch in [(-.2,4,'Without conic','///'),(.2,5,'Without simultaneous generation','...')]:
    ax.bar(x+offset,np.maximum(0,d[:,col]),.4,label=label,hatch=hatch,color='white',edgecolor='black',linewidth=.6)
ax.set(xlabel='Instance',ylabel='Increase in recovery cost (%)',xticks=x,xticklabels=d[:,0].astype(int))
ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);ax.legend(loc='upper left')
save(fig,'sensitivity_simultaneous')
