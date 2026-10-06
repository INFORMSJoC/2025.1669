# Copyright (c) 2026 Zhouchun Huang.
# SPDX-License-Identifier: MIT
"""Build eight Markdown tables from manuscript values and archived raw files.

Run from any directory: python scripts/summarize_results.py
No solver, external runfolder, pandas or network connection is needed.
"""
import argparse
import json
import re
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
R=ROOT/'results'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,default=R,help='Directory for generated tables (default: results)')
args=parser.parse_args()
OUT=args.output_dir.resolve()
paper_source=json.loads((Path(__file__).resolve().parent/'data/reported_values.json').read_text(encoding='utf-8'))
paper=paper_source['tables']


def read_kpi(path):
    values={}
    for line in path.read_text().splitlines():
        m=re.match(r'^([^:]+):\s*([-+\d.eE]+)\s*$',line)
        if m: values[m[1].strip()]=float(m[2])
    return values


def show(v):
    if v is None:return '*'
    if isinstance(v,str):return v.replace('|','\\|')
    return f'{v:,.8g}'


def table(headers, rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','|'+'|'.join('---' for _ in headers)+'|']+
        ['| '+' | '.join(show(v) for v in row)+' |' for row in rows])+'\n'


def write(path, text):
    p=OUT/path;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text,encoding='utf-8',newline='\n')


recovery=[];integrality=[]
for i in range(1,31):
    t=paper['computational'][str(i)];u=paper['csc'][str(i)]
    assert u[0]==t[2] and u[3]==t[5], f'Inconsistent no-CSC manuscript columns: {i}'
    k=read_kpi(R/f'computational_comparison/raw/{i}/CCG/kpi.out')
    metrics=['# Cancelled','# Swaps','# Delayed','Ttl delay(min)']
    recovery.append([i]+[k[m] for m in metrics])
    integrality.append([i,100*k['IP gap']])

perf_headers=['Instance','CG-I seconds','CG-II seconds','CCG seconds','CG-I cost (1000)','CG-II cost (1000)','CCG cost (1000)']
csc_headers=['Instance','No CSC seconds','CQMIP-CSC seconds','CCG-CSC seconds','No CSC cost (1000)','CQMIP-CSC cost (1000)','CCG-CSC cost (1000)','CSC flights','Savings (%)']
rec_headers=['Instance','Cancelled','Swaps','Delayed','Total delay (min)']
write('supplement/recovery_strategies/summary.md','# CCG recovery strategies\n\nTable S4. Raw files: `../../computational_comparison/raw/<instance>/CCG/`. Total delay uses the KPI output precision.\n\n'+table(rec_headers,recovery))

# The primary summaries follow the manuscript exactly; raw KPI files and their
# independently calculated summaries remain available as evidence.
reported_performance=[[i]+paper['computational'][str(i)] for i in range(1,31)]
reported_csc=[[i]+paper['csc'][str(i)] for i in range(1,31)]
reported_integrality=[[i,round(value,2) or 0.0] for i,value in integrality]
reported_perf_headers=perf_headers+['Reported optimality gap (%)']
reported_csc_headers=csc_headers[:7]+['Reported optimality gap (%)','CSC flights','Savings (%)']
notice='Source: manuscript-reported values. `*` retains the manuscript placeholder. Costs are in thousands; percentages are in percent units.'
write('computational_comparison/summary.md','# Computational comparison: manuscript values\n\n'+notice+'\n\n'+table(reported_perf_headers,reported_performance))
write('cruise_speed_control/summary.md','# CSC comparison: manuscript values\n\n'+notice+'\n\n'+table(reported_csc_headers,reported_csc))
write('computational_comparison/integrality.md','# Restricted-master integrality gaps: manuscript precision\n\nArchived ratios rounded to two decimal places, consistent with the manuscript narrative: instance 26 is 0.14%, and the remaining instances round to 0.00%. Instance identities come from the archive.\n\n'+table(['Instance','Restricted LP-IP gap (%)'],[[i,f'{value:.2f}'] for i,value in reported_integrality]))

mk=[]
for m in [1,2,5,10,20,100,10000]:
    k=read_kpi(R/f'sensitivity/max_copies/raw/{m}/CCG-CSC/kpi.out')
    mk.append([m,k['Total']/1000,k['Run time(sec)']])
write('sensitivity/max_copies/summary.md','# Maximum-copy sensitivity\n\nInstance 1, with CSC. The manuscript infinity label is implemented as an upper bound of 10000.\n\n'+table(['Maximum copies','Cost (1000)','Seconds'],mk))
sim=[]
for i in range(1,31):
    ks=[read_kpi(R/f'sensitivity/simultaneous/raw/{s}/{i}/CCG-CSC/kpi.out') for s in ['original','no_conic','no_multiple']]
    costs=[k['Total'] for k in ks]
    sim.append([i]+[c/1000 for c in costs]+[100*(c-costs[0])/costs[0] for c in costs[1:]])
write('sensitivity/simultaneous/summary.md','# Simultaneous-generation sensitivity\n\nThe source figure clips negative increases to zero; this table retains the signed values. Source: archived sensitivity runs.\n\n'+table(['Instance','Original cost (1000)','No conic cost (1000)','No multiple cost (1000)','No conic increase (%)','No multiple increase (%)'],sim))

iterations=[[float(v) for v in line.split()] for line in (R/'copy_generation/raw/copy_gen.out').read_text().splitlines() if line.strip() and not line.startswith('#')]
assert len(iterations)==35 and all(len(row)==12 for row in iterations)
assert all(row[1]==sum(row[2:6]) and row[6]==sum(row[7:11]) for row in iterations)
write('copy_generation/summary.md','# Copy-generation iteration history\n\nAll 35 iterations from the source log. Generated and retained totals equal the sum of their four components. Objective is in original cost units. The source log does not identify its instance.\n\n'+table(['Iteration','Generated','Individual regular','Multiple regular','Individual CSC','Multiple CSC','Retained','Retained individual regular','Retained multiple regular','Retained individual CSC','Retained multiple CSC','Objective'],iterations))

example=(R/'supplement/N281AK/raw/CCG/result.out').read_text()
section=re.search(r'^N281AK.*?\n(.*?)(?=^N\S+\[|\Z)',example,re.M|re.S).group(1)
legs=re.findall(r'^(\d+)-d(\d+)\((\d+/\d+)([A-Z]{3})(\d+:\d+)---([A-Z]{3})(\d+:\d+)\)',section,re.M)
paper_legs=[re.sub(r'\\\\.*','',line).strip().split('&') for line in (R/'supplement/N281AK/path_for_N281AK.tex').read_text().splitlines() if re.match(r'^\s*\d+\s*&',line)]
assert len(legs)==len(paper_legs)==10
path_rows=[]
for leg,reported in zip(legs,paper_legs):
    published=[v.strip() for v in reported]
    flight,delay,date,origin,atd,dest,ata=leg
    assert int(delay)==int(published[8]) and origin==published[2] and dest==published[3]
    assert atd.zfill(5)==published[6].zfill(5) and ata.zfill(5)==published[7].zfill(5)
    assert flight==published[1] or (flight=='69001' and published[1]=='69')
    path_rows.append(published[:2]+[flight]+published[2:])
write('supplement/N281AK/summary.md','# N281AK path in instance 30\n\nTable S3. Times are UTC; internal leg 69001 is published as flight 69.\n\n'+table(['Leg','Published flight','Internal ID','Origin','Destination','STD','STA','ATD','ATA','Delay (min)'],path_rows))

print('Generated eight result tables in',OUT)
