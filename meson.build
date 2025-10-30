project(
	'aylp_tl_piezo',
	'c',
	default_options: ['c_std=gnu17', 'warning_level=3', 'optimization=3']
)

incdir = include_directories(['libaylp'])

sp_dep = dependency('libserialport')
gsl_dep = dependency('gsl')
json_dep = dependency('json-c')
deps = [sp_dep, gsl_dep, json_dep]

shared_library('aylp_tl_piezo', 'aylp_tl_piezo.c',
	name_prefix: '',
	dependencies: deps,
	include_directories: incdir,
	install: true,
	install_dir: '/opt/anyloop',
	override_options: 'b_lundef=false'
)
