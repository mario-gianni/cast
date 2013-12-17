/*
 * Parser.hpp
 *
 *  Created on: Dec 29, 2011
 *      Author: alcor
 */

#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <planning_roma.hpp>
#include <topograph.hpp>
#include <EmbeddedEclipse.hpp>
#include <robot_position.hpp>
#include <are.hpp>
#include <diagnostic.hpp>

using namespace eu::nifti::Planning::slice;
using namespace eu::nifti::env::topograph;
using namespace eu::nifti::env::position;
using namespace eu::nifti::env::are;
using namespace eu::nifti::env::diagnostic;


class Parser {
public:
	Parser();
	virtual ~Parser();

	GoToNodeActionPtr eclipse2IceGoToNodeAction(EC_word);
	TopoGraphWriterActionPtr eclipse2IceTopoGraphWriterAction(EC_word);
	TogoGraphBuilderActionPtr eclipse2IceTopoGraphBuilderAction(EC_word);
	FunctionalMappingActionPtr eclipse2IceFuncMappingAction(EC_word);
	RotatingLaserActionPtr eclipse2IceRotatingLaserAction(EC_word);
	FlipperActionPtr eclipse2IceFlipperAction(EC_word);
	DifferentialActionPtr eclipse2IceDifferentialAction(EC_word);
	GapDetectionActionPtr eclipse2IceGapDetectionAction(EC_word);
	CenterLaserActionPtr eclipse2IceCenterLaserAction(EC_word);
	MoveBaseActionPtr eclipse2IceMoveBaseAction(EC_word);
	AutoModeActionPtr eclipse2IceAutoModeAction(EC_word);

	EC_word node(NodePtr);
	EC_word edge(EdgePtr);
	EC_word in(CurrentPosPtr);
	EC_word origin(BasePosPtr);
	EC_word at(PosePtr);
	EC_word artefact(ArtefactPtr);
	EC_word battery(BatteryStatusPtr);
	EC_word wifi(WiFiStatusPtr);

	EC_word wifiStrength(CurrentPosPtr,WiFiStatusPtr);

	EC_word ice2EclipseGotoNodeAction(GoToNodeActionPtr);
	EC_word ice2EclipseTopoGraphWriterAction(TopoGraphWriterActionPtr);
	EC_word ice2EclipseTopoGraphBuilderAction(TogoGraphBuilderActionPtr);
	EC_word ice2EclipseFunctMappingAction(FunctionalMappingActionPtr);
	EC_word ice2EclipseRotatingLaserAction(RotatingLaserActionPtr);
	EC_word ice2EclipseFlipperAction(FlipperActionPtr);
	EC_word ice2EclipseDifferentialAction(DifferentialActionPtr);
	EC_word ice2EclipseGapDetectionAction(GapDetectionActionPtr);
	EC_word ice2EclipseCenterLaserAction(CenterLaserActionPtr);
	EC_word ice2EclipseMoveBaseAction(MoveBaseActionPtr);
	EC_word ice2EclipseAutoModeAction(AutoModeActionPtr);
};

#endif /* PARSER_HPP_ */
