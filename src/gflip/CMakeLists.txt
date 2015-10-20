SET(gflip_SRCS 
  gflip_engine.cpp
  gflip_bow_generator.cpp
  gflip_feature_extractor.cpp

) 

SET(gflip_HDRS
  gflip_types.hpp
  gflip_engine.hpp
  gflip_bow_generator.hpp
  gflip_feature_extractor.hpp

)

ADD_LIBRARY(gflip SHARED ${gflip_SRCS})
TARGET_LINK_LIBRARIES(gflip vocabulary feature geometry sensorstream sensors utils boost_filesystem boost_serialization)
ADD_DEPENDENCIES(gflip flirt)

install(TARGETS gflip
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME})

install(FILES ${gflip_HDRS} DESTINATION include/${PROJECT_NAME}/gflip)
