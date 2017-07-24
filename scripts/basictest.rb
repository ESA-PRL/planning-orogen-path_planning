require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.run 'path_planning::Task' => 'path_planning' do

  path_planning = Orocos.name_service.get 'path_planning'
  path_planning.configure

  path_planning.start

  Readline::readline("Press ENTER to exit\n")

end
