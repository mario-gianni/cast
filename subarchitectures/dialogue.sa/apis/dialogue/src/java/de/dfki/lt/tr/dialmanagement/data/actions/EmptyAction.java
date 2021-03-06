// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package de.dfki.lt.tr.dialmanagement.data.actions;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Empty action - does not do anything
 * (this action is essentially used during policy construction)
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class EmptyAction extends AbstractAction {

	public EmptyAction(String id) {
		super(id);
	}

	@Override
	public dFormula asFormula() {
		return null;
	}

	@Override
	public void fillArguments(HashMap<String, dFormula> arguments)
			throws DialogueException {		
	}

	@Override
	public Collection<String> getUnderspecifiedArguments() {
		return new LinkedList<String>();
	}

	@Override
	public boolean isUnderspecified() {
		return false;
	}
	
	@Override
	public String toString() {
		return "";
	}

}
