Import('global_env')

env = global_env.Clone()
Export('env')

env.FeatureSConscript(dirs=['lib-rc'])

env.BuiltInObject(global_env)
