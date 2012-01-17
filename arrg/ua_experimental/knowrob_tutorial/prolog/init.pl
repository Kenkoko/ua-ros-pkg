%%
%% Copyright (C) 2010 by Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(mod_vis).
:- register_ros_package(ias_semantic_map).
:- register_ros_package(comp_spatial).
:- register_ros_package(comp_temporal).
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_tutorial).

:- use_module(library('tabletop_obj')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('/Users/anton/ros-e/ua-ros-pkg/ua_experimental/knowrob_tutorial/owl/tabletop_obj.owl', false, false, true).
:- rdf_db:rdf_register_ns(tabletop_obj, 'http://ias.cs.tum.edu/kb/tabletop_obj.owl#', [keep(true)]).

:- owl_parser:owl_parse('/Users/anton/ros-e/ua-ros-pkg/ua_experimental/knowrob_tutorial/owl/coffeecup.owl', false, false, true).
:- rdf_db:rdf_register_ns(tabletop_obj, 'http://ias.cs.tum.edu/kb/coffeecup.owl#', [keep(true)]).
