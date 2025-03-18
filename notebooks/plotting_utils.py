#!/usr/bin/env python3

__all__=['setup_formatting','pull_max','process_data','plot_data','load_data','load_final_data','reject_outliers_fn','process_eval_data','load_final_data_alt','load_rew_buff','process_data_alt']

import pickle
import numpy as np
import glob
import matplotlib.pyplot as plt
import matplotlib.font_manager
import seaborn as sns
sns.set(style="whitegrid")

def setup_formatting(size=2):
    # figure formatting
    if size==0 : # small
        font_sizes, fig_size = [10,12,13], (8,3)  
    elif size==1 : # medium
        font_sizes, fig_size = [11,13,14], (12,4.5) 
    elif size==2 : # medium single
        font_sizes, fig_size = [11,13,14], (6,4.5) 
    elif size==3 : # medium single
        font_sizes, fig_size = [12,16,18], (16,6) 
        font_sizes = np.array(font_sizes)*1.05
    elif size==4 : # medium single
        font_sizes, fig_size = [12,16,18], (16,6) 
        font_sizes = np.array(font_sizes)*1.45    
    else: # large
        font_sizes, fig_size = [12,14,15], (16,6) 
    print(font_sizes)
    # plt.rcParams.update({'font.size': font_sizes[0], 'font.family': 'sans-serif', 'font.sans-serif': 'Arial', 
    plt.rcParams.update({'font.size': font_sizes[0], 'font.family': 'Times New Roman',# 'font.sans-serif': 'Arial', 
                         'legend.fontsize': font_sizes[0],
                         'xtick.labelsize': font_sizes[1], 'ytick.labelsize': font_sizes[1], 
                         'axes.labelsize': font_sizes[2], 'axes.titlesize': font_sizes[2], 'figure.titlesize': font_sizes[2],'figure.figsize': fig_size})
    # plt.rcParams.keys()

def pull_max(d,xs,start_idx):
    _rew = None
    new_mean = []
    for m,x in zip(d,xs):
        if _rew is None:
            _rew = m
        else:
            if m > _rew and x > start_idx:
                _rew = m
        new_mean.append(_rew)
    return new_mean

def process_data(env_path, N=60,get_max=False, start_idx=0, penalize_early_stop=False, skip_early_stop=False, max_steps=1000, data_file='reward_data.pkl'):
    # load data sets
    data = []
    min_size = np.inf
    for i,path in enumerate(sorted(glob.glob(env_path + 'seed*/'))):
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            data_set = np.stack(data_set)
            if data_set[-1,0] < min_size:
                if not(np.isinf(min_size)):
                    print(path,data_set[-1,0])
                min_size = data_set[-1,0]
            data.append(data_set)
        except:
            pass
#     print(f'{i+1} seeds',env_path)
        
    # process data sets (skip early termination and pull max values if specified)
    interp_data, raw_data, interp_steps = [], [], []
    xy_idx = [0, 1]
    x_samples = np.linspace(start_idx, min_size, N)
    for data_set in data:
        _x, _y = data_set[:,xy_idx].T 
        try: 
            _steps = data_set[:,3] # if saved steps (added after some simulations were run)
        except: 
            _steps = np.roll(np.roll(data_set[:,0],-1 ) - data_set[:,0],1)
            _steps[0] = _x[0]            
        if skip_early_stop:
            data_idx = (_steps >= max_steps)
            _x, _y = _x[data_idx], _y[data_idx]
        elif penalize_early_stop: 
            penalty = max_steps - _steps
            _y = _y - penalty
        if get_max:
            _y = pull_max(_y,_x,start_idx)
            
        interp_data.append(np.hstack([np.zeros(1),np.interp(x_samples[1:], _x, _y)]))
        if not skip_early_stop:
            interp_steps.append(np.interp(x_samples, _x, _steps))
        raw_data.append([_x, _y])
            
    if not skip_early_stop:
        # save processed data to log
        mean = np.mean(interp_steps, axis=0)
        std  = np.std(interp_steps, axis=0)
        steps_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
                   'max' : np.max(interp_steps, axis=0),
                   'min' : np.min(interp_steps, axis=0),
                   '-std' : mean-std, '+std' : mean+std, 
                    'data' : interp_steps}
    else: 
        steps_log = {}
    
    # save processed data to log
    mean = np.mean(interp_data, axis=0)
    std  = np.std(interp_data, axis=0)
    data_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
               'max' : np.max(interp_data, axis=0),
               'min' : np.min(interp_data, axis=0),
               '-std' : mean-std, '+std' : mean+std, 
                'data' : interp_data, 'raw_data' : raw_data,
               'steps_log': steps_log, 'debug': np.sum(np.vstack(interp_data)[:,-1]<-900)}
    
    return data_log

def process_data_alt(env_path, N=60,get_max=False, start_idx=0, penalize_early_stop=False, skip_early_stop=False, max_steps=1000, data_file='reward_data.pkl'):
    # load data sets
    data = []
    min_size = np.inf
    max_size = 0
    for i,path in enumerate(sorted(glob.glob(env_path + 'seed*/'))):
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            data_set = np.stack(data_set)
            if data_set[-1,0] < min_size:
                if not(np.isinf(min_size)):
                    print(path,data_set[-1,0])
                min_size = data_set[-1,0]
            if data_set[-1,0] > max_size:
                max_size = data_set[-1,0]
            data.append(data_set)
        except:
            pass
#     print(f'{i+1} seeds',env_path)
        
    # process data sets (skip early termination and pull max values if specified)
    interp_data, raw_data, interp_steps = [], [], []
    xy_idx = [0, 1]
    x_samples = np.linspace(start_idx, max_size, N)
    for data_set in data:
        _x, _y = data_set[:,xy_idx].T 
        try: 
            _steps = data_set[:,3] # if saved steps (added after some simulations were run)
        except: 
            _steps = np.roll(np.roll(data_set[:,0],-1 ) - data_set[:,0],1)
            _steps[0] = _x[0]            
        if skip_early_stop:
            data_idx = (_steps >= max_steps)
            _x, _y = _x[data_idx], _y[data_idx]
        elif penalize_early_stop: 
            penalty = max_steps - _steps
            _y = _y - penalty
        if get_max:
            _y = pull_max(_y,_x,start_idx)
            
        interp_data.append(np.hstack([np.zeros(1),np.interp(x_samples[1:], _x, _y)]))
        if not skip_early_stop:
            interp_steps.append(np.interp(x_samples, _x, _steps))
        raw_data.append([_x, _y])
            
    if not skip_early_stop:
        # save processed data to log
        mean = np.mean(interp_steps, axis=0)
        std  = np.std(interp_steps, axis=0)
        steps_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
                   'max' : np.max(interp_steps, axis=0),
                   'min' : np.min(interp_steps, axis=0),
                   '-std' : mean-std, '+std' : mean+std, 
                    'data' : interp_steps}
    else: 
        steps_log = {}
    
    # save processed data to log
    mean = np.mean(interp_data, axis=0)
    std  = np.std(interp_data, axis=0)
    data_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
               'max' : np.max(interp_data, axis=0),
               'min' : np.min(interp_data, axis=0),
               '-std' : mean-std, '+std' : mean+std, 
                'data' : interp_data, 'raw_data' : raw_data,
               'steps_log': steps_log, 'debug': np.sum(np.vstack(interp_data)[:,-1]<-900)}
    
    return data_log

def plot_data(ax, log, label, color, fill=True, raw=True, zorder=0, linestyle='solid',linewidth=1,marker=None,use_ep=False):
    all_data = np.array(log['data']).T
    if use_ep:
        log['x']/=1000.
    if not fill:
        ax.plot(log['x'],all_data,alpha=0.3,color=color,zorder=zorder,linestyle=linestyle,marker=marker)
        if raw: 
            for data in log['raw_data']:
                if use_ep:
                    data[0]/=1000.
                ax.plot(data[0],data[1],alpha=0.3,color=color,zorder=zorder,linestyle=linestyle,marker=marker)
    ax.plot(log['x'], log['mean'], label=label,color=color,zorder=zorder,linestyle=linestyle,linewidth=linewidth,marker=marker)
    if fill: 
        ax.fill_between(log['x'], 
                        log['-std'], log['+std'],
                        alpha=0.2,color=color,zorder=zorder)
        

        
from matplotlib.patches import Patch
from scipy import stats
def load_data(env_path, data_file='reward_data.pkl'):
    # load data sets
    data = []
    max_size = 0
    for i,path in enumerate(sorted(glob.glob(env_path + 'seed*/'))):
        print(path)
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            data_set = np.stack(data_set)
            if data_set.shape[0] > max_size:
                max_size = data_set.shape[0]
            data.append(data_set)
        except:
            pass

    # print([f'{label}: {dim}' for label, dim in zip(['seeds','iters'], np.array(data).shape)]+[env_path])
    xy_idx = [0, 1]
    y_data = np.zeros((len(data),max_size))
    for idx,data_set in enumerate(data):
        _x, _y = data_set[:,xy_idx].T 
        y_data[idx,:len(_y)] = _y
    y_data = y_data[~np.isnan(y_data)]
    
    # save processed data to log
    x_samples = 1.
    mean = np.mean(y_data)
    std  = np.std(y_data)
    data_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
               'max' : np.max(y_data),
               'min' : np.min(y_data),
               '-std' : mean-std, '+std' : mean+std,'data':y_data}
    
    return data_log


from matplotlib.patches import Patch
def load_rew_buff(env_path, data_file='buffer_data.pkl',reward_scale=1.,rew_idx=13):
    # load data sets
    data = []
    seed = []
    max_size = 0
    for i,path in enumerate(sorted(glob.glob(env_path + '/seed*/'))):
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            ceil_buff = (np.ceil(data_set.shape[0]/1000)*1000).astype(int)
            max_buff = (np.floor(data_set.shape[0]/1000)*1000).astype(int)
            x = np.zeros(ceil_buff)
            x[:max_buff] = data_set[:,rew_idx]
            x = np.sum(x.reshape(-1,1000),1)/reward_scale
            if x.shape[0] > max_size:
                max_size = x.shape[0]
            data.append(x)
            seed.append(path.split('seed_')[-1].split('/')[0])
        except:
            pass

    print([f'{label}: {dim}' for label, dim in zip(['seeds','iters'], [len(data),len(seed)])]+[env_path])
    y_idx = [0] # rew
    y_data = np.zeros((len(data),max_size))
    for idx,data_set in enumerate(data):
        y_data[idx,:len(data_set)] = data_set
    y_data = y_data[~np.isnan(y_data)]
    
    if len(seed) > 1:
        flip = seed.pop(1)
        seed.insert(0,flip)
        y_data = y_data.reshape(len(seed),-1)
        order = np.arange(len(seed))
        order[0] = 1
        order[1] = 0
        y_data = y_data[order]
        y_data = y_data.reshape(-1)
    elif len(seed) == 0:
        print('error',env_path)
        
    # save processed data to log
    x_samples = 1.
    mean = np.mean(y_data)
    std  = np.std(y_data)
    data_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
               'max' : np.max(y_data),
               'min' : np.min(y_data),
               '-std' : mean-std, '+std' : mean+std,'data':y_data}
    
    return seed,data_log

def load_final_data_alt(env_path, data_file='buffer_data.pkl',num_avg=5,max_x=False,max_rew=False):
    # load data sets
    data = []
    seed = []
    for i,path in enumerate(sorted(glob.glob(env_path + '/seed*/'))):
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            if max_x: 
                data.append(np.max(data_set[:,0])) # state, action, reward, next_state, next_action, done
            elif max_rew:
                data.append(np.max(data_set[:,13])) # state (11), action (2), reward (1), next_state (11), next_action (2), done (1)
            else:
                ceil_buff = (np.ceil(data_set.shape[0]/1000)*1000).astype(int)
                max_buff = (np.floor(data_set.shape[0]/1000)*1000).astype(int)
                x = np.ones(ceil_buff)
                x[:max_buff] = data_set[:max_buff,0].reshape(-1,1000)
                data.append(np.mean(x[-num_avg:,-1]))
            seed.append(path.split('seed_')[-1].split('/')[0])
        except:
            pass
    if len(seed) > 1:
        flip = seed.pop(1)
        seed.insert(0,flip)
        flip = data.pop(1)
        data.insert(0,flip)
    elif len(seed) == 0:
        print('error',env_path)
    return seed,data

def load_final_data(env_path, data_file='buffer_data.pkl'):
    # load data sets
    data = []
    seed = []
    for i,path in enumerate(sorted(glob.glob(env_path + 'seed*/'))):
        try: 
            data_set = pickle.load(open(path + data_file, 'rb'))
            data.append(np.stack(data_set)[-1][0])
            seed.append(path.split('seed_')[-1].split('/')[0])
        except:
            pass
    if len(seed) > 1:
        flip = seed.pop(1)
        seed.insert(0,flip)
        flip = data.pop(1)
        data.insert(0,flip)
    return seed,data


def reject_outliers_fn(x, iq_range=50):
    pcnt = (100 - iq_range) / 2
    iqr = np.subtract(*np.percentile(x, [100-pcnt, pcnt]))
    median = np.median(x)
    return np.abs(x - median) <= iqr

def process_eval_data(env_path, penalize_early_stop=False, skip_early_stop=True, reject_outliers=True, max_steps=1000, data_file='reward_data.pkl'):
    # load data sets
    data = []
    for i,path in enumerate(sorted(glob.glob(env_path + 'seed*/'))):
        # if '513' not in path:
        try: 
            with open(path + data_file, 'rb') as f:
                data_set = pickle.load(f)
            data.append(np.stack(data_set))
        except:
            pass

    y_data = np.empty(0)
    xy_idx = [0, 1]
    for data_set in data:
        _, _y = data_set[:,xy_idx].T 
        _steps = data_set[:,3] # if saved steps (added after some simulations were run)
        if skip_early_stop:
            data_idx = (_steps >= max_steps)
            _y = _y[data_idx]
        elif penalize_early_stop: 
            penalty = (max_steps - _steps)
            _y = _y - penalty
        y_data = np.hstack([y_data,_y])

                
    y_data = y_data[~np.isnan(y_data)]
    if reject_outliers: 
        locs = reject_outliers_fn(y_data,80) # 68,95,99.7 = 1,2,3 sigma
        y_data  = y_data[locs]

    # if ( len(data) > 0 and len(data) < 10 ) or (y_data.shape[0]  > 0 and y_data.shape[0] < 1000): 
    #     print(len(data),y_data.shape,env_path.split('/')[-3:])    
        
    # save processed data to log
    x_samples = 1.
    mean = np.mean(y_data)
    std  = np.std(y_data)
    data_log = {'x' : x_samples, 'mean' : mean, 'std' : std, 
               # 'max' : np.max(y_data), 'min' : np.min(y_data),
               '-std' : mean-std, '+std' : mean+std,'data':y_data}
    
    return data_log